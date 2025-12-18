import os
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
import uuid
from sentence_transformers import SentenceTransformer
import numpy as np
import hashlib

class QdrantVectorDB:
    def __init__(self):
        # Check if we should use local or cloud Qdrant
        local_qdrant = os.getenv("LOCAL_QDRANT", "true").lower() == "true"

        if local_qdrant:
            # Use in-memory Qdrant for local development
            self.client = QdrantClient(":memory:")
        else:
            # Connect to Qdrant Cloud
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")

            if not qdrant_url or not qdrant_api_key:
                raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set when using cloud Qdrant")

            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                port=6333,
                https=True
            )

        self.collection_name = "physical_ai_book"
        self.model = SentenceTransformer('all-MiniLM-L6-v2')  # Free sentence transformer model

        # Initialize the collection
        self._init_collection()

    def _init_collection(self):
        """Initialize the Qdrant collection"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),  # 384 for all-MiniLM-L6-v2
            )

    def add_document_chunks(self, chunks: List[Dict]):
        """Add document chunks to the vector database"""
        points = []

        for chunk in chunks:
            # Generate embedding for the content
            embedding = self.model.encode(chunk['content']).tolist()

            # Create a unique ID for the chunk
            chunk_id = str(uuid.uuid4())

            point = PointStruct(
                id=chunk_id,
                vector=embedding,
                payload={
                    "content": chunk['content'],
                    "document_id": chunk.get('document_id', ''),
                    "title": chunk.get('title', ''),
                    "source": chunk.get('source', ''),
                    "metadata": chunk.get('metadata', {})
                }
            )
            points.append(point)

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search(self, query: str, top_k: int = 5) -> List[Dict]:
        """Search for relevant chunks based on the query"""
        # Generate embedding for the query
        query_embedding = self.model.encode(query).tolist()

        # Search in Qdrant
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k
        )

        results = []
        for result in search_results:
            results.append({
                'content': result.payload['content'],
                'metadata': {
                    'document_id': result.payload.get('document_id', ''),
                    'title': result.payload.get('title', ''),
                    'source': result.payload.get('source', ''),
                },
                'score': result.score
            })

        return results

    def clear_collection(self):
        """Clear all points in the collection (useful for testing)"""
        try:
            self.client.delete_collection(self.collection_name)
        except:
            pass
        self._init_collection()