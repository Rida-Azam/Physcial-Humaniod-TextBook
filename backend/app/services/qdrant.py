import qdrant_client
from qdrant_client.http import models
from typing import List, Dict, Any
import os
from openai import OpenAI

class QdrantService:
    def __init__(self):
        # Initialize Qdrant client - in production, use cloud instance
        self.client = qdrant_client.QdrantClient(
            url=os.getenv("QDRANT_URL", "http://localhost:6333")
        )
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.collection_name = "textbook_content"
        self._ensure_collection()

    def _ensure_collection(self):
        """
        Ensure the collection exists with proper configuration
        """
        try:
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1536,  # OpenAI embedding size
                    distance=models.Distance.COSINE
                )
            )

    def _get_embedding(self, text: str) -> List[float]:
        """
        Get embedding for text using OpenAI
        """
        response = self.openai_client.embeddings.create(
            input=text,
            model="text-embedding-ada-002"
        )
        return response.data[0].embedding

    async def search(self, query_text: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for relevant documents based on query
        """
        query_embedding = self._get_embedding(query_text)

        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        results = []
        for result in search_results:
            results.append({
                "content": result.payload.get("content", ""),
                "source": result.payload.get("source", ""),
                "title": result.payload.get("title", ""),
                "score": result.score
            })

        return results

    async def add_document(self, content: str, source: str, title: str, doc_id: str = None):
        """
        Add a document to the vector database
        """
        embedding = self._get_embedding(content)

        if doc_id is None:
            import uuid
            doc_id = str(uuid.uuid4())

        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=doc_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        "source": source,
                        "title": title
                    }
                )
            ]
        )