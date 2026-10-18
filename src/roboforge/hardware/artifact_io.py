"""Bounded JSON data loading shared by portable hardware artifacts."""

import json

from roboforge.core import finite


def decode_json_object(raw: bytes, *, maximum_bytes: int, label: str) -> dict:
    if len(raw) > maximum_bytes:
        raise ValueError(f"{label} exceeds its {maximum_bytes}-byte limit")

    def mapping(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate {label} key: {key}")
            result[key] = value
        return result

    def reject_constant(value):
        raise ValueError(f"nonfinite JSON constant: {value}")

    value = json.loads(
        raw,
        object_pairs_hook=mapping,
        parse_constant=reject_constant,
        parse_float=lambda number: finite(float(number), f"{label} number"),
    )
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value
