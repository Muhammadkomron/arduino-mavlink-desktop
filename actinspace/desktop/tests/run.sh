#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
FAILED=0

echo "========================================"
echo "  NazarX Ground Station - Test Runner"
echo "========================================"
echo ""

# --- Go Unit Tests ---
echo "--- Go Unit Tests ---"
echo ""
cd "$ROOT_DIR"
if go test ./tests/backend/unit/ -v -count=1; then
    echo ""
    echo "[PASS] Go unit tests passed"
else
    echo ""
    echo "[FAIL] Go unit tests failed"
    FAILED=1
fi

echo ""
echo "--- Go Integration Tests ---"
echo ""

if go test ./tests/backend/integration/ -v -count=1; then
    echo ""
    echo "[PASS] Go integration tests passed"
else
    echo ""
    echo "[FAIL] Go integration tests failed"
    FAILED=1
fi

echo ""
echo "--- Go E2E Tests ---"
echo ""

if go test ./tests/backend/e2e/ -v -count=1 -race; then
    echo ""
    echo "[PASS] Go E2E tests passed"
else
    echo ""
    echo "[FAIL] Go E2E tests failed"
    FAILED=1
fi

echo ""
echo "--- Frontend Unit Tests ---"
echo ""

cd "$ROOT_DIR/frontend"

# Install deps if node_modules is missing
if [ ! -d "node_modules" ]; then
    echo "Installing frontend dependencies..."
    npm install --silent
fi

if npx vitest run --dir src/__tests__ --exclude 'src/__tests__/integration/**' --exclude 'src/__tests__/e2e/**'; then
    echo ""
    echo "[PASS] Frontend unit tests passed"
else
    echo ""
    echo "[FAIL] Frontend unit tests failed"
    FAILED=1
fi

echo ""
echo "--- Frontend Integration Tests ---"
echo ""

if npx vitest run --dir src/__tests__/integration; then
    echo ""
    echo "[PASS] Frontend integration tests passed"
else
    echo ""
    echo "[FAIL] Frontend integration tests failed"
    FAILED=1
fi

echo ""
echo "--- Frontend E2E Tests ---"
echo ""

if npx vitest run --dir src/__tests__/e2e; then
    echo ""
    echo "[PASS] Frontend E2E tests passed"
else
    echo ""
    echo "[FAIL] Frontend E2E tests failed"
    FAILED=1
fi

echo ""
echo "--- Frontend Build ---"
echo ""

if npm run build; then
    echo ""
    echo "[PASS] Frontend build succeeded"
else
    echo ""
    echo "[FAIL] Frontend build failed"
    FAILED=1
fi

echo ""
echo "========================================"
if [ $FAILED -eq 0 ]; then
    echo "  ALL TESTS PASSED"
else
    echo "  SOME TESTS FAILED"
fi
echo "========================================"

exit $FAILED
