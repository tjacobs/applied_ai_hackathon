#!/usr/bin/env bash
set -euo pipefail

# Simple launcher for TutorBot web UI + server
# - Starts FastAPI (uvicorn) with hot reload on port 8080
# - Launches Chromium in kiosk mode to the local URL

PORT="8080"
HOST="0.0.0.0"
APP="main:app"
URL="http://localhost:${PORT}"

# Ensure dependencies exist (skip venv; system python/pip assumed)
if ! python3 -c "import fastapi, uvicorn, jinja2" >/dev/null 2>&1; then
  echo "[TutorBot] Installing Python dependencies..." >&2
  python3 -m pip install --user -r requirements.txt
fi

# Kill any existing uvicorn bound to our app/port (best-effort, ignore errors)
if command -v pkill >/dev/null 2>&1; then
  pkill -f "uvicorn .*${APP}.*${PORT}" >/dev/null 2>&1 || true
fi

# Start server in the background
echo "[TutorBot] Starting server on ${HOST}:${PORT}..." >&2
python3 -m uvicorn ${APP} --host ${HOST} --port ${PORT} --reload &
SERVER_PID=$!

# Trap to cleanup server on exit
cleanup() {
  if kill -0 "${SERVER_PID}" >/dev/null 2>&1; then
    kill "${SERVER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

# Wait briefly for server to come up
sleep 2

# Launch Chromium in kiosk
BROWSER_CMD="chromium-browser"
if ! command -v "${BROWSER_CMD}" >/dev/null 2>&1; then
  # Fallback to chromium or google-chrome if available
  if command -v chromium >/dev/null 2>&1; then BROWSER_CMD=chromium; fi
  if command -v google-chrome >/dev/null 2>&1; then BROWSER_CMD=google-chrome; fi
fi

echo "[TutorBot] Launching kiosk at ${URL} using ${BROWSER_CMD}..." >&2
"${BROWSER_CMD}" --kiosk --noerrdialogs --disable-infobars --incognito "${URL}"

# Keep script attached to uvicorn (so CTRL+C stops both)
wait ${SERVER_PID}
