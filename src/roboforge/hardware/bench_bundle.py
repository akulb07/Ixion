"""Portable original bench inputs and independently recomputed acceptance."""

import hashlib
import io
import json
import stat
import struct
import zipfile
import zlib
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from roboforge import __version__
from roboforge.config import Nonnegative, Real, Schema

from .artifact_io import decode_json_object
from .bench_acceptance import BenchAcceptancePolicy, check_bench_comparison
from .bench_trace import compare_bench_trace, decode_bench_trace

MIB = 1024 * 1024
ENTRY_LIMITS = {
    "run.json": 64 * MIB,
    "trace.csv": 4 * MIB,
    "metadata.json": 65_536,
    "policy.json": 65_536,
    "alignment.json": 16_384,
    "acceptance.json": 131_072,
    "manifest.json": 65_536,
}
MAX_TOTAL_BYTES = 69 * MIB
MAX_ARCHIVE_BYTES = 70 * MIB
SCOPE = "Original input bytes and recomputed recorded-data checks; no simulation rerun, signature or authentication of hardware measurements"


class BenchBundleAlignment(Schema):
    schema_version: Literal[1] = 1
    time_offset_s: Real
    maximum_sample_age_s: Nonnegative
    case_name: str | None = Field(default=None, min_length=1, max_length=200)

    @model_validator(mode="after")
    def case_label(self):
        if self.case_name is not None and not self.case_name.strip():
            raise ValueError("selected case must contain text")
        return self


def _json(value):
    return (json.dumps(value, indent=2, allow_nan=False) + "\n").encode("utf-8")


def _decode(payloads, name):
    return decode_json_object(payloads[name], maximum_bytes=ENTRY_LIMITS[name], label=name)


def _evaluate(payloads):
    run = _decode(payloads, "run.json")
    policy = BenchAcceptancePolicy.model_validate(_decode(payloads, "policy.json"))
    alignment = BenchBundleAlignment.model_validate(_decode(payloads, "alignment.json"))
    trace = decode_bench_trace(payloads["trace.csv"], payloads["metadata.json"])
    comparison = compare_bench_trace(
        run,
        trace,
        time_offset_s=alignment.time_offset_s,
        maximum_sample_age_s=alignment.maximum_sample_age_s,
        case_name=alignment.case_name,
        report_sha256=hashlib.sha256(payloads["run.json"]).hexdigest(),
    )
    return check_bench_comparison(
        comparison,
        policy,
        policy_source_sha256=hashlib.sha256(payloads["policy.json"]).hexdigest(),
    )


def _summary(result):
    return {key: value for key, value in result.items() if key != "comparison"}


def _manifest(payloads, result, version):
    return {
        "schema_version": 1,
        "artifact": "rover_bench_bundle",
        "ixion_version": version,
        "status": result["status"],
        "source_kind": result["source_kind"],
        "files": {
            name: {"sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)}
            for name, raw in sorted(payloads.items())
        },
        "scope": SCOPE,
    }


def write_bench_bundle(
    path: str | Path,
    run_path: str | Path,
    trace_path: str | Path,
    metadata_path: str | Path,
    policy_path: str | Path,
    *,
    time_offset_s: float,
    maximum_sample_age_s: float,
    case_name: str | None = None,
) -> dict:
    if Path(path).exists():
        raise ValueError("bench bundle output already exists; choose a new path")
    payloads = {}
    for name, source in zip(
        ("run.json", "trace.csv", "metadata.json", "policy.json"),
        (run_path, trace_path, metadata_path, policy_path),
    ):
        with Path(source).open("rb") as stream:
            raw = stream.read(ENTRY_LIMITS[name] + 1)
        if len(raw) > ENTRY_LIMITS[name]:
            raise ValueError(f"bench bundle input exceeds size limit: {name}")
        payloads[name] = raw
    alignment = BenchBundleAlignment(
        time_offset_s=time_offset_s, maximum_sample_age_s=maximum_sample_age_s, case_name=case_name
    )
    payloads["alignment.json"] = _json(alignment.model_dump(mode="json"))
    result = _evaluate(payloads)
    payloads["acceptance.json"] = _json(_summary(result))
    payloads["manifest.json"] = _json(_manifest(payloads, result, __version__))
    if any(len(raw) > ENTRY_LIMITS[name] for name, raw in payloads.items()):
        raise ValueError("bench bundle generated member exceeds its size limit")
    if sum(map(len, payloads.values())) > MAX_TOTAL_BYTES:
        raise ValueError("bench bundle exceeds total size limit")
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, raw in sorted(payloads.items()):
            entry = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            entry.external_attr = 0o100644 << 16
            archive.writestr(entry, raw)
    if buffer.tell() > MAX_ARCHIVE_BYTES:
        raise ValueError("bench bundle exceeds compressed size limit")
    with Path(path).open("xb") as stream:
        stream.write(buffer.getvalue())
    return result


def verify_bench_bundle(path: str | Path) -> dict:
    try:
        with Path(path).open("rb") as stream:
            raw = stream.read(MAX_ARCHIVE_BYTES + 1)
        if len(raw) > MAX_ARCHIVE_BYTES:
            raise ValueError("bench bundle exceeds compressed input limit")
        # Bound central-directory allocation before ZipFile constructs member objects.
        if len(raw) < 22:
            raise ValueError("bench bundle has no complete ZIP footer")
        signature, disk, directory_disk, disk_count, count, directory_size, offset, comment = (
            struct.unpack("<4s4H2LH", raw[-22:])
        )
        if (
            signature != b"PK\x05\x06"
            or disk
            or directory_disk
            or disk_count != 7
            or count != 7
            or comment
            or directory_size > 65_536
            or offset + directory_size != len(raw) - 22
        ):
            raise ValueError(
                "bench bundle requires a bounded seven-file ZIP without comments or ZIP64"
            )
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            entries = archive.infolist()
            names = [entry.filename for entry in entries]
            if len(entries) != len(ENTRY_LIMITS) or set(names) != set(ENTRY_LIMITS):
                raise ValueError("bench bundle requires exactly the seven named regular files")
            if any(
                entry.flag_bits & 1
                or entry.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED)
                or stat.S_IFMT(entry.external_attr >> 16) not in (0, stat.S_IFREG)
                for entry in entries
            ):
                raise ValueError("bench bundle contains encrypted or unsupported entries")
            if (
                any(entry.file_size > ENTRY_LIMITS[entry.filename] for entry in entries)
                or sum(entry.file_size for entry in entries) > MAX_TOTAL_BYTES
            ):
                raise ValueError("bench bundle exceeds entry or total uncompressed size limits")
            payloads = {}
            for entry in entries:
                with archive.open(entry) as member:
                    content = member.read(entry.file_size + 1)
                if len(content) != entry.file_size:
                    raise ValueError("bench bundle member length disagrees with its header")
                payloads[entry.filename] = content
        manifest = _decode(payloads, "manifest.json")
        if (
            type(manifest.get("schema_version")) is not int
            or manifest["schema_version"] != 1
            or manifest.get("artifact") != "rover_bench_bundle"
            or not isinstance(manifest.get("ixion_version"), str)
            or not 1 <= len(manifest["ixion_version"]) <= 100
            or not manifest["ixion_version"].strip()
        ):
            raise ValueError("unsupported bench bundle manifest")
        members = {name: raw for name, raw in payloads.items() if name != "manifest.json"}
        hashes = {
            name: {"sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)}
            for name, raw in sorted(members.items())
        }
        if manifest.get("files") != hashes:
            raise ValueError("bench bundle checksum, size or file membership mismatch")
        result = _evaluate(payloads)
        if _decode(payloads, "acceptance.json") != _summary(result):
            raise ValueError("saved bench acceptance disagrees with recomputed input evidence")
        if manifest != _manifest(members, result, manifest["ixion_version"]):
            raise ValueError("bench bundle manifest disagrees with recomputed evidence")
        return result
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        AttributeError,
        zipfile.BadZipFile,
        RuntimeError,
        RecursionError,
        zlib.error,
        EOFError,
    ) as exc:
        raise ValueError(f"Invalid bench bundle {path}: {exc}") from exc
