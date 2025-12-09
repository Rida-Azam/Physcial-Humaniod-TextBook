import asyncio
from typing import Dict, List, Any, Optional
from app.services.qdrant import QdrantService
from openai import AsyncOpenAI
import os

class RAGService:
    def __init__(self):
        self.qdrant_service = QdrantService()
        self.openai_client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    async def query(self, query_text: str, user_id: Optional[str] = None, context: Dict = {}) -> Dict[str, Any]:
        """
        Process a query using RAG (Retrieval Augmented Generation)
        """
        try:
            # Step 1: Retrieve relevant documents from vector database
            relevant_docs = await self.qdrant_service.search(query_text)

            # Step 2: Prepare context for LLM
            context_str = "\n".join([doc.get("content", "") for doc in relevant_docs])

            # Step 3: Generate response using OpenAI
            response = await self.openai_client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context. Be accurate, educational, and helpful."
                    },
                    {
                        "role": "user",
                        "content": f"Context: {context_str}\n\nQuestion: {query_text}\n\nPlease provide a detailed, accurate answer based on the context provided."
                    }
                ],
                temperature=0.3,
                max_tokens=1000
            )

            # Step 4: Prepare response
            answer = response.choices[0].message.content
            sources = [doc.get("source", "") for doc in relevant_docs]
            confidence = min(len(relevant_docs) * 0.25, 1.0)  # Simple confidence calculation

            return {
                "answer": answer,
                "sources": sources,
                "confidence": confidence,
                "retrieved_docs_count": len(relevant_docs)
            }

        except Exception as e:
            raise Exception(f"RAG query failed: {str(e)}")