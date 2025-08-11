import asyncio
import json
import random
import uuid
import threading
from pathlib import Path
from move_head import nod, shake, look_around
from speak import speak
from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

app = FastAPI(title="TutorBot Server")

# Mount static assets
app.mount("/static", StaticFiles(directory="static"), name="static")

# Templates
templates = Jinja2Templates(directory="templates")


class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self._lock:
            self.active_connections.append(websocket)

    async def disconnect(self, websocket: WebSocket):
        async with self._lock:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        # Send to all clients; remove closed connections
        stale: List[WebSocket] = []
        for conn in list(self.active_connections):
            try:
                await conn.send_json(message)
            except Exception:
                stale.append(conn)
        for s in stale:
            await self.disconnect(s)


manager = ConnectionManager()


@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "title": "TutorBot · Math Game",
        },
    )


QUESTIONS: List[dict] = []


def _load_questions() -> List[dict]:
    data_path = Path(__file__).parent / "data" / "questions.json"
    try:
        with data_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        # basic validation and normalization
        result = []
        for q in data:
            if not isinstance(q, dict):
                continue
            prompt = q.get("prompt")
            choices = q.get("choices")
            answer = q.get("answer")
            if prompt is None or not isinstance(choices, list):
                continue
            result.append({
                "prompt": str(prompt),
                "choices": list(choices),
                "answer": answer,
                "tts": q.get("tts"),
            })
        return result
    except Exception:
        return []


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    # Per-connection session state
    score = 0
    total = 0  # we will increment only on correct answers to reflect progress
    current = None  # dict with id, prompt, answer, choices
    index = 0  # pointer into QUESTIONS

    def next_problem():
        nonlocal index
        if index >= len(QUESTIONS):
            return None
        q = QUESTIONS[index]
        # Defensive copy and assign runtime id
        payload = {
            "id": str(uuid.uuid4()),
            "prompt": q.get("prompt"),
            "choices": list(q.get("choices", [])),
            "answer": q.get("answer"),
            "tts": q.get("tts"),
        }
        return payload

    try:
        # Send initial state
        await websocket.send_json({
            "type": "hello",
            "payload": {
                "message": "Connected to TutorBot",
            },
        })
        # Send initial score
        await websocket.send_json({
            "type": "score",
            "payload": {"score": score, "total": total},
        })
        # Do not auto-send a problem; wait for a 'start' command
        if not QUESTIONS:
            await websocket.send_json({
                "type": "feedback",
                "payload": {"ok": False, "text": "No questions available."},
            })
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            mtype = msg.get("type")
            payload = msg.get("payload", {})

            if mtype == "start":
                if QUESTIONS:
                    # Reset progression if requested
                    if payload.get("reset"):
                        index = 0
                        score = 0
                        total = 0
                    current = next_problem()
                    if current is not None:
                        # Make the robot look around
                        try:
                            look_around()
                            # Speak the question text in a non-blocking way
                            question_text = current.get("prompt", "")
                            if question_text:
                                def speak_question():
                                    try:
                                        speak(question_text)
                                    except Exception as e:
                                        print(f"Error speaking question: {e}")
                                threading.Thread(target=speak_question, daemon=True).start()
                        except Exception as e:
                            print(f"Error in question display: {e}")
                        await websocket.send_json({
                            "type": "problem",
                            "payload": {"id": current["id"], "prompt": current["prompt"], "choices": current["choices"], "tts": current.get("tts")},
                        })
                continue

            if mtype == "answer" and isinstance(payload, dict):
                val = payload.get("value")
                pid = payload.get("id")
                if current and pid == current["id"]:
                    ok = (val == current["answer"])  # exact match
                    if ok:
                        score += 1
                        total += 1
                        # Make the robot nod
                        try:
                            nod()
                        except Exception as e:
                            print(f"Error making robot nod: {e}")
                        # Send feedback with updated score
                        await websocket.send_json({
                            "type": "feedback",
                            "payload": {
                                "ok": True,
                                "text": "Correct!",
                                "score": score,
                                "total": total,
                            },
                        })
                        # Advance to next question
                        index += 1
                        nxt = next_problem()
                        if nxt is None:
                            await websocket.send_json({
                                "type": "complete",
                                "payload": {"text": "All questions complete!"},
                            })
                            # Do not close the socket; allow reconnect or idle
                        else:
                            current = nxt
                            # Speak the next question
                            question_text = current.get("prompt", "")
                            if question_text:
                                def speak_question():
                                    try:
                                        speak(question_text)
                                    except Exception as e:
                                        print(f"Error speaking question: {e}")
                                threading.Thread(target=speak_question, daemon=True).start()
                            await websocket.send_json({
                                "type": "problem",
                                "payload": {"id": current["id"], "prompt": current["prompt"], "choices": current["choices"], "tts": current.get("tts")},
                            })
                    else:
                        # Make the robot shake for incorrect answer
                        try:
                            shake()
                        except Exception as e:
                            print(f"Error making robot shake: {e}")
                        # Incorrect: show feedback but do NOT advance or change totals
                        await websocket.send_json({
                            "type": "feedback",
                            "payload": {
                                "ok": False,
                                "text": "Try again",
                                "score": score,
                                "total": total,
                            },
                        })
                else:
                    # Ignore if out-of-sync
                    continue
            else:
                # ignore other messages for now
                pass
    except WebSocketDisconnect:
        await manager.disconnect(websocket)
    except Exception:
        # Ensure disconnect on any error
        await manager.disconnect(websocket)


@app.post("/publish")
async def publish(message: dict):
    # Allows server-side or external processes to push events to UI
    await manager.broadcast({
        "type": message.get("type", "event"),
        "payload": message.get("payload", {}),
    })
    return {"ok": True}


# Optional: background heartbeat to verify push works in kiosk
async def _heartbeat_task():
    while True:
        await asyncio.sleep(10)
        await manager.broadcast({
            "type": "heartbeat",
            "payload": {"ts": asyncio.get_event_loop().time()},
        })


@app.on_event("startup")
async def on_startup():
    # Load questions from disk
    global QUESTIONS
    QUESTIONS = _load_questions()
    asyncio.create_task(_heartbeat_task())


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
