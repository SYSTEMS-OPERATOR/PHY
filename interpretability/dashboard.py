from __future__ import annotations

from fastapi import FastAPI
from fastapi.responses import JSONResponse

app = FastAPI()


@app.get("/status")
async def status() -> JSONResponse:
    return JSONResponse({"status": "ok"})
