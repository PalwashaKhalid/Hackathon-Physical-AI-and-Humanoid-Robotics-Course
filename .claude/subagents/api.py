#!/usr/bin/env python3
"""
API for Claude Code Subagents System
Provides endpoints to trigger and manage subagents
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import Dict, List, Any, Optional
import asyncio
import json
from datetime import datetime
import os

import sys
import os
# Add the subagents directory to the path to import the orchestrator
subagents_dir = os.path.dirname(__file__)
sys.path.append(subagents_dir)
from subagent_orchestrator import SubagentOrchestrator

app = FastAPI(
    title="Claude Code Subagents API",
    description="API for managing and triggering Claude Code Subagents in the Physical AI & Humanoid Robotics Book project",
    version="1.0.0"
)

# Initialize orchestrator
orchestrator = SubagentOrchestrator()

# Pydantic models
class SkillExecutionRequest(BaseModel):
    skill_id: str
    parameters: Dict[str, Any]

class EventTriggerRequest(BaseModel):
    event_type: str
    event_data: Dict[str, Any]

class SkillResponse(BaseModel):
    success: bool
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    timestamp: str

@app.get("/")
async def root():
    return {"message": "Claude Code Subagents API for Physical AI & Humanoid Robotics Book"}

@app.get("/api/skills")
async def list_skills():
    """List all available skills."""
    return {
        "skills": list(orchestrator.skill_registry.keys()),
        "count": len(orchestrator.skill_registry),
        "categories": list(set(
            skill_info['config'].get('category', 'unknown')
            for skill_info in orchestrator.skill_registry.values()
        ))
    }

@app.post("/api/skill/trigger", response_model=SkillResponse)
async def trigger_skill(request: SkillExecutionRequest):
    """Trigger a specific skill with parameters."""
    try:
        result = await orchestrator.execute_skill(request.skill_id, request.parameters)
        return SkillResponse(
            success=result['success'],
            result=result.get('result'),
            error=result.get('error'),
            timestamp=result['timestamp']
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/event/trigger")
async def trigger_event(request: EventTriggerRequest):
    """Trigger skills based on an event."""
    try:
        results = await orchestrator.trigger_event(request.event_type, request.event_data)
        return {
            "event_type": request.event_type,
            "triggered_skills": len(results),
            "results": results,
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/status")
async def get_status():
    """Get the status of the subagents system."""
    return {
        "status": "running",
        "timestamp": datetime.now().isoformat(),
        "total_skills": len(orchestrator.skill_registry),
        "config_loaded": bool(orchestrator.config),
        "last_activity": "2025-12-18T12:00:00Z"  # Would be dynamic in real implementation
    }

@app.get("/api/config")
async def get_config():
    """Get the current configuration."""
    return orchestrator.config

# Health check endpoint
@app.get("/health")
async def health_check():
    """Simple health check."""
    return {"status": "healthy", "timestamp": datetime.now().isoformat()}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8081)