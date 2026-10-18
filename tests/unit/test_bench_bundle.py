import hashlib
import json
import stat
import struct
import zipfile

import pytest

from roboforge.cli import main
from roboforge.hardware import bench_bundle
from roboforge.hardware.bench_bundle import verify_bench_bundle, write_bench_bundle
from tests.unit.test_bench_acceptance import policy_data
from tests.unit.test_bench_trace import metadata, recording


def inputs(tmp_path):
    root = tmp_path / "inputs"
    root.mkdir()
    run, trace, meta, policy = [
        root / name for name in ("run.json", "trace.csv", "metadata.json", "policy.json")
    ]
    run.write_text(json.dumps(recording()))
    trace.write_text("time_s,speed_m_s\n0,0.1\n0.1,1.1\n0.2,2.1\n")
    meta.write_text(json.dumps(metadata()))
    policy.write_text(json.dumps(policy_data()))
    return run, trace, meta, policy


def write(path, sources):
    return write_bench_bundle(path, *sources, time_offset_s=0, maximum_sample_age_s=0)


def rewrite(source, destination, mutate, *, rehash=False):
    with zipfile.ZipFile(source) as archive:
        payloads = {name: archive.read(name) for name in archive.namelist()}
    mutate(payloads)
    if rehash:
        manifest = json.loads(payloads["manifest.json"])
        manifest["files"] = {
            name: {"sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)}
            for name, raw in payloads.items()
            if name != "manifest.json"
        }
        payloads["manifest.json"] = json.dumps(manifest).encode()
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, raw in payloads.items():
            archive.writestr(name, raw)


def test_bundle_retains_original_bytes_and_recomputes_identical_acceptance(tmp_path):
    sources = inputs(tmp_path)
    original = [source.read_bytes() for source in sources]
    first, second = tmp_path / "first.zip", tmp_path / "second.zip"
    result = write(first, sources)
    assert result["status"] == "pass" and result["source_kind"] == "synthetic"
    assert verify_bench_bundle(first) == result
    write(second, sources)
    assert first.read_bytes() == second.read_bytes()
    with zipfile.ZipFile(first) as archive:
        assert len(archive.namelist()) == 7
        assert [
            archive.read(name) for name in ("run.json", "trace.csv", "metadata.json", "policy.json")
        ] == original
        assert "comparison" not in json.loads(archive.read("acceptance.json"))
    with pytest.raises(ValueError, match="already exists"):
        write(first, sources)
    assert original == [source.read_bytes() for source in sources]


def test_changed_source_bytes_fail_hashes_and_rehashed_data_still_requires_recomputed_results(
    tmp_path,
):
    source = tmp_path / "source.zip"
    write(source, inputs(tmp_path))

    def change(payloads):
        payloads["trace.csv"] = b"time_s,speed_m_s\n0,99\n0.1,99\n0.2,99\n"

    altered = tmp_path / "altered.zip"
    rewrite(source, altered, change)
    with pytest.raises(ValueError, match="checksum"):
        verify_bench_bundle(altered)
    rewritten = tmp_path / "rehash.zip"
    rewrite(source, rewritten, change, rehash=True)
    with pytest.raises(ValueError, match="recomputed input evidence"):
        verify_bench_bundle(rewritten)


def test_edited_acceptance_cannot_become_a_pass_even_after_rehashing(tmp_path):
    sources = inputs(tmp_path)
    policy = policy_data()
    policy["source_kind"] = "measured"
    sources[-1].write_text(json.dumps(policy))
    source = tmp_path / "failed.zip"
    assert write(source, sources)["status"] == "fail"
    assert verify_bench_bundle(source)["status"] == "fail"

    def change(payloads):
        acceptance = json.loads(payloads["acceptance.json"])
        acceptance["status"] = "pass"
        acceptance["checks"] = []
        payloads["acceptance.json"] = json.dumps(acceptance).encode()

    altered = tmp_path / "altered.zip"
    rewrite(source, altered, change, rehash=True)
    with pytest.raises(ValueError, match="recomputed"):
        verify_bench_bundle(altered)


@pytest.mark.parametrize(
    "kind",
    ["extra", "missing", "path", "duplicate", "symlink", "compression", "comment", "truncated"],
)
def test_unsupported_archive_shapes_are_rejected_without_extraction(tmp_path, kind):
    source, altered = tmp_path / "source.zip", tmp_path / "altered.zip"
    write(source, inputs(tmp_path))
    with zipfile.ZipFile(source) as archive:
        payloads = {name: archive.read(name) for name in archive.namelist()}
    if kind == "truncated":
        altered.write_bytes(source.read_bytes()[:-12])
    else:
        if kind == "missing":
            payloads.pop("trace.csv")
        if kind == "path":
            payloads["../trace.csv"] = payloads.pop("trace.csv")
        if kind == "extra":
            payloads["extra.txt"] = b"unexpected"
        with zipfile.ZipFile(altered, "w") as archive:
            for name, raw in payloads.items():
                entry = zipfile.ZipInfo(name)
                if kind == "symlink" and name == "trace.csv":
                    entry.external_attr = (stat.S_IFLNK | 0o777) << 16
                if kind == "compression":
                    entry.compress_type = zipfile.ZIP_BZIP2
                archive.writestr(entry, raw)
            if kind == "duplicate":
                with pytest.warns(UserWarning):
                    archive.writestr("trace.csv", b"duplicate")
            if kind == "comment":
                archive.comment = b"unsupported comment"
    with pytest.raises(ValueError, match="Invalid bench bundle"):
        verify_bench_bundle(altered)
    assert not (tmp_path / "trace.csv").exists()


def test_size_limits_and_directory_count_are_checked_before_member_decoding(tmp_path, monkeypatch):
    source = tmp_path / "source.zip"
    write(source, inputs(tmp_path))
    monkeypatch.setitem(bench_bundle.ENTRY_LIMITS, "trace.csv", 2)
    with pytest.raises(ValueError, match="uncompressed"):
        verify_bench_bundle(source)
    monkeypatch.setitem(bench_bundle.ENTRY_LIMITS, "trace.csv", 4 * 1024 * 1024)
    raw = bytearray(source.read_bytes())
    struct.pack_into("<H", raw, len(raw) - 22 + 10, 65_535)
    wrong_count = tmp_path / "wrong-count.zip"
    wrong_count.write_bytes(raw)
    with pytest.raises(ValueError, match="bounded seven-file"):
        verify_bench_bundle(wrong_count)
    monkeypatch.setattr(bench_bundle, "MAX_ARCHIVE_BYTES", 10)
    with pytest.raises(ValueError, match="compressed input"):
        verify_bench_bundle(source)


def test_archive_member_reads_remain_bounded_during_decompression(tmp_path, monkeypatch):
    source = tmp_path / "source.zip"
    write(source, inputs(tmp_path))
    original_read = zipfile.ZipExtFile.read
    reads = []

    def bounded_read(member, size=-1):
        assert 0 <= size <= bench_bundle.ENTRY_LIMITS[member.name] + 1
        reads.append(member.name)
        return original_read(member, size)

    monkeypatch.setattr(zipfile.ZipExtFile, "read", bounded_read)
    assert verify_bench_bundle(source)["status"] == "pass"
    assert set(reads) == set(bench_bundle.ENTRY_LIMITS)


def test_cli_verifies_failed_evidence_and_exports_only_requested_files(tmp_path):
    sources = inputs(tmp_path)
    policy = policy_data()
    policy["source_kind"] = "measured"
    sources[-1].write_text(json.dumps(policy))
    archive = tmp_path / "evidence.zip"
    args = [
        "rover-bench-bundle",
        *map(str, sources),
        "--time-offset-s",
        "0",
        "--max-sample-age-s",
        "0",
        "--output",
        str(archive),
    ]
    assert main(args) == 3
    before = archive.read_bytes()
    output, junit = tmp_path / "verified.json", tmp_path / "verified.xml"
    assert (
        main(["verify-bench-bundle", str(archive), "--output", str(output), "--junit", str(junit)])
        == 3
    )
    assert json.loads(output.read_text())["status"] == "fail"
    assert archive.read_bytes() == before and not (tmp_path / "trace.csv").exists()
    assert main(["verify-bench-bundle", str(archive), "--output", str(output)]) == 2
    assert main(["verify-bench-bundle", str(archive), "--output", str(archive)]) == 2
    assert archive.read_bytes() == before
