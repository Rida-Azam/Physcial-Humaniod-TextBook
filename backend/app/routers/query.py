from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from app.services.rag import RAGService

router = APIRouter()
rag_service = RAGService()

class QueryRequest(BaseModel):
    query: str
    user_id: str = None
    context: dict = {}

class QueryResponse(BaseModel):
    answer: str
    sources: list = []
    confidence: float = 0.0

@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    try:
        result = await rag_service.query(request.query, request.user_id, request.context)
        return QueryResponse(
            answer=result.get("answer", ""),
            sources=result.get("sources", []),
            confidence=result.get("confidence", 0.0)
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))