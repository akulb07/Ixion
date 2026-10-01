"""Compare recorded runs without rerunning simulations or discarding failures."""

import hashlib
import json
import math
from datetime import UTC, datetime

from pydantic import Field, model_validator

from roboforge import __version__
from roboforge.config import Schema
from roboforge.service import ACTIVE, READY, RunService, ServiceError


class ComparisonRequest(Schema):
    run_ids: tuple[str, ...] = Field(min_length=2, max_length=4)

    @model_validator(mode="after")
    def unique_runs(self):
        if len(set(self.run_ids)) != len(self.run_ids):
            raise ValueError("select distinct runs; the first run is the baseline")
        return self


def _differences(documents, path=""):
    """Keep arrays together so large paths do not create thousands of table rows."""
    if all(document == documents[0] for document in documents[1:]):
        return []
    if all(isinstance(document, dict) for document in documents):
        rows = []
        for key in sorted(set().union(*(document.keys() for document in documents))):
            child = f"{path}.{key}" if path else key
            if all(key in document for document in documents):
                rows.extend(_differences([document[key] for document in documents], child))
            else:
                rows.append(
                    {
                        "path": child,
                        "values": [
                            {"present": key in document, "value": document.get(key)}
                            for document in documents
                        ],
                    }
                )
        return rows
    return [{"path": path, "values": [{"present": True, "value": d} for d in documents]}]


def compare_runs(service: RunService, specification: ComparisonRequest) -> dict:
    runs = []
    for run_id in specification.run_ids:
        record = service.get(run_id)
        if record["status"] in ACTIVE:
            raise ServiceError("wait for selected runs to finish before comparing")
        try:
            config_bytes = service.artifact(run_id, "config.json").read_bytes()
            config = json.loads(config_bytes)
            canonical = json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False)
            if hashlib.sha256(canonical.encode()).hexdigest() != record["config_sha256"]:
                raise ValueError("configuration checksum mismatch")
            metrics = None
            if record["status"] in READY:
                manifest = json.loads(service.artifact(run_id, "manifest.json").read_bytes())
                for filename in ("config.json", "job.json", "metrics.json"):
                    data = service.artifact(run_id, filename).read_bytes()
                    if hashlib.sha256(data).hexdigest() != manifest["files"][filename]:
                        raise ValueError("artifact checksum mismatch")
                metrics = json.loads(service.artifact(run_id, "metrics.json").read_bytes())
                if metrics != record["metrics"] or not all(
                    isinstance(value, (int, float))
                    and not isinstance(value, bool)
                    and math.isfinite(value)
                    for value in metrics.values()
                ):
                    raise ValueError("invalid recorded metrics")
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
            raise ServiceError(f"run {run_id} failed comparison integrity validation") from exc
        runs.append({**record, "config": config, "metrics": metrics})

    metrics = []
    names = sorted({name for run in runs for name in (run["metrics"] or {})})
    for name in names:
        values = [(run["metrics"] or {}).get(name) for run in runs]
        deltas = []
        for value in values:
            delta = value - values[0] if value is not None and values[0] is not None else None
            deltas.append(delta if delta is None or math.isfinite(delta) else None)
        metrics.append({"name": name, "values": values, "deltas": deltas})

    differences = _differences([run["config"] for run in runs])
    sources = [(run.get("provenance") or {}).get("source", {}).get("sha256") for run in runs]
    dependencies = [(run.get("provenance") or {}).get("dependencies") for run in runs]
    return {
        "format_version": 1,
        "software_version": __version__,
        "created_utc": datetime.now(UTC).isoformat(),
        "baseline_id": specification.run_ids[0],
        "runs": runs,
        "metrics": metrics,
        "config_differences": differences,
        "same_setup": not any(row["path"] != "name" for row in differences),
        "same_software": len({run["software_version"] for run in runs}) == 1,
        "same_source": len(set(sources)) == 1 if all(sources) else None,
        "same_dependencies": all(item == dependencies[0] for item in dependencies)
        if all(dependencies)
        else None,
    }
