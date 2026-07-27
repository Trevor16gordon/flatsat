#!/usr/bin/env python3
"""Requirements traceability: match declared requirements to verifying tests.

Reads ``requirements/*.toml``, collects the test suite, and cross-references
``@pytest.mark.verifies("ID")`` markers against declared IDs. Prints a matrix
and, with ``--strict``, fails the build on either kind of break:

  * a requirement whose verification method is "test" with no test marked
    against it — an unverified claim wearing a verified badge;
  * a test citing an ID that no requirement declares — a dangling reference
    that hides the requirement it was supposed to cover.

Requirements verified by analysis, inspection, or demonstration are listed
with their evidence but not expected to have tests: pretending a 10-minute
cyclictest campaign is a unit test would be worse than naming the method.

Usage:
  ~/venvs/flatsat-ml/bin/python tools/traceability.py
  ~/venvs/flatsat-ml/bin/python tools/traceability.py --strict
"""

from __future__ import annotations

import argparse
import io
import sys
from contextlib import redirect_stdout
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:  # Python 3.11+
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib

REPO_ROOT = Path(__file__).resolve().parents[1]
REQUIREMENTS_DIR = REPO_ROOT / "requirements"
TESTS_DIR = REPO_ROOT / "flight" / "tests"
TEST_VERIFIED = "test"


@dataclass(frozen=True)
class Requirement:
    """One declared requirement.

    Attributes:
        id: Stable identifier.
        statement: What must be true.
        verification: Method — test, analysis, inspection, demonstration.
        evidence: Where the proof lives.
        status: draft, allocated, verified, or waived.
        source: File the requirement was declared in.
    """

    id: str
    statement: str
    verification: str
    evidence: str
    status: str
    source: str


def load_requirements(directory: Path = REQUIREMENTS_DIR) -> list[Requirement]:
    """Load every requirement declared under a directory.

    Args:
        directory: Directory containing requirement TOML files.

    Returns:
        All requirements, sorted by ID.

    Raises:
        ValueError: If two files declare the same ID.
    """
    found: dict[str, Requirement] = {}
    for path in sorted(directory.glob("*.toml")):
        data: dict[str, Any] = tomllib.loads(path.read_text())
        for row in data.get("requirement", []):
            req = Requirement(
                id=str(row["id"]),
                statement=" ".join(str(row["statement"]).split()),
                verification=str(row["verification"]),
                evidence=str(row.get("evidence", "")),
                status=str(row.get("status", "draft")),
                source=path.name,
            )
            if req.id in found:
                raise ValueError(f"duplicate requirement id {req.id!r} in {path.name}")
            found[req.id] = req
    return sorted(found.values(), key=lambda r: r.id)


def collect_markers(tests_dir: Path = TESTS_DIR) -> dict[str, list[str]]:
    """Collect ``verifies`` markers from the test suite.

    Args:
        tests_dir: Directory to collect tests from.

    Returns:
        Mapping of requirement ID to the test node names citing it.
    """
    import pytest

    # Running this file directly puts tools/ on sys.path, not the repo root,
    # so the test modules' `import flight...` would fail and collect nothing.
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))

    citations: dict[str, list[str]] = {}

    class MarkerCollector:
        """Pytest plugin capturing verifies() markers during collection."""

        def pytest_collection_modifyitems(self, items: list[Any]) -> None:
            """Record each collected item's verifies markers.

            Args:
                items: Collected test items.
            """
            for item in items:
                for marker in item.iter_markers("verifies"):
                    for req_id in marker.args:
                        citations.setdefault(str(req_id), []).append(item.name)

    # Collection output is noise here; keep it out of the report.
    sink = io.StringIO()
    with redirect_stdout(sink):
        code = pytest.main(
            ["--collect-only", "-q", "--no-header", "-p", "no:cacheprovider", str(tests_dir)],
            plugins=[MarkerCollector()],
        )
    output = sink.getvalue()
    if int(code) != 0:
        # Never silently report "no tests verify this" when the real cause is
        # a collection failure — that would turn a broken suite into a
        # traceability report full of false gaps.
        raise RuntimeError(f"test collection failed (exit {int(code)}):\n{output[-2000:]}")
    return citations


def main() -> int:
    """Print the traceability matrix and optionally enforce it.

    Returns:
        0 when consistent (or not --strict); 1 on traceability breaks.
    """
    parser = argparse.ArgumentParser(description="Requirements traceability matrix.")
    parser.add_argument("--strict", action="store_true", help="exit non-zero on breaks")
    args = parser.parse_args()

    requirements = load_requirements()
    citations = collect_markers()
    declared = {req.id for req in requirements}

    print(f"{'REQUIREMENT':<16} {'METHOD':<14} {'STATUS':<10} VERIFIED BY")
    print("-" * 96)
    unverified: list[Requirement] = []
    for req in requirements:
        tests = citations.get(req.id, [])
        if req.verification == TEST_VERIFIED:
            detail = ", ".join(tests) if tests else "*** NO TEST ***"
            if not tests:
                unverified.append(req)
        else:
            detail = f"({req.verification}) {req.evidence[:60]}"
        print(f"{req.id:<16} {req.verification:<14} {req.status:<10} {detail}")

    dangling = {rid: names for rid, names in citations.items() if rid not in declared}

    print("-" * 96)
    by_method: dict[str, int] = {}
    for req in requirements:
        by_method[req.verification] = by_method.get(req.verification, 0) + 1
    summary = ", ".join(f"{count} {method}" for method, count in sorted(by_method.items()))
    print(f"{len(requirements)} requirements ({summary}); {len(citations)} cited by tests")

    if unverified:
        print(f"\nUNVERIFIED ({len(unverified)}) — declared test-verified but no test cites them:")
        for req in unverified:
            print(f"  {req.id}  {req.statement[:70]}")
    if dangling:
        print(f"\nDANGLING ({len(dangling)}) — tests cite undeclared requirement IDs:")
        for rid, names in sorted(dangling.items()):
            print(f"  {rid}  cited by {', '.join(names)}")

    if args.strict and (unverified or dangling):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
