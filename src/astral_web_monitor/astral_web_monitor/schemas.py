"""Pydantic models for REST request/response envelopes.

Follows the rob_station convention: REST returns {ok, message, data} for
success and HTTP 409 with {detail} for command conflicts.
"""
from __future__ import annotations

from typing import Any

from pydantic import BaseModel


class ApiEnvelope(BaseModel):
    ok: bool
    message: str = ""
    data: Any = None


class StartRequest(BaseModel):
    preset: str


class PresetInfo(BaseModel):
    name: str
    package: str
    launch: str
    args: dict[str, str] = {}
    description: str = ""
