from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, Dict, Any

router = APIRouter()

class PersonalizeRequest(BaseModel):
    user_id: str
    preferences: Dict[str, Any]

class PersonalizeResponse(BaseModel):
    message: str
    updated_preferences: Dict[str, Any]

class ProgressRequest(BaseModel):
    user_id: str
    chapter_id: str
    progress: float
    completed: bool = False

class ProgressResponse(BaseModel):
    message: str
    progress_updated: bool

@router.post("/preferences", response_model=PersonalizeResponse)
async def set_preferences(request: PersonalizeRequest):
    try:
        # In a real implementation, this would save to Neon PostgreSQL
        return PersonalizeResponse(
            message="Preferences updated successfully",
            updated_preferences=request.preferences
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/progress", response_model=ProgressResponse)
async def update_progress(request: ProgressRequest):
    try:
        # In a real implementation, this would save progress to Neon PostgreSQL
        return ProgressResponse(
            message="Progress updated successfully",
            progress_updated=True
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))