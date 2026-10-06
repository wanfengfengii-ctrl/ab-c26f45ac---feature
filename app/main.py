"""HTTP surface for the trajectory audit service."""

from __future__ import annotations

import os

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from . import __version__
from .audit import AuditInputError, run_audit
from .retime import RetimeConflictError, run_retime
from .schemas import AuditRequest, AuditResponse, RetimeConflict, RetimeRequest, RetimeResponse

# The health-check path is configurable so the same image can be probed at
# whatever route the deployment environment expects.
HEALTH_PATH = os.environ.get("API_HEALTH_PATH", "/api/health")

app = FastAPI(
    title="Robot Trajectory Audit Service",
    version=__version__,
    description=(
        "Audits piecewise cubic Bézier joint trajectories exactly (no "
        "sampling): travel, velocity and acceleration limits are adjudicated "
        "over the whole continuous curve with exact decimal arithmetic."
    ),
)


@app.exception_handler(AuditInputError)
async def audit_input_error_handler(_request: Request, exc: AuditInputError) -> JSONResponse:
    return JSONResponse(status_code=422, content={"detail": exc.errors})


@app.exception_handler(RetimeConflictError)
async def retime_conflict_error_handler(_request: Request, exc: RetimeConflictError) -> JSONResponse:
    return JSONResponse(
        status_code=409,
        content=RetimeConflict(reason=exc.reason, msg=exc.msg).model_dump(),
    )


@app.get(HEALTH_PATH)
async def health() -> dict:
    return {"status": "ok"}


@app.post("/api/trajectories/audit", response_model=AuditResponse)
async def audit_trajectory(request: AuditRequest) -> AuditResponse:
    return run_audit(request)


@app.post(
    "/api/trajectories/retime",
    response_model=RetimeResponse,
    responses={409: {"model": RetimeConflict, "description": "Trajectory cannot be scheduled within max_scale"}},
)
async def retime_trajectory(request: RetimeRequest) -> RetimeResponse:
    return run_retime(request)
