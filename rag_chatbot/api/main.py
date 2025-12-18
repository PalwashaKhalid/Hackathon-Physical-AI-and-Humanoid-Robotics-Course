from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import sys
import glob
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from vector_db.qdrant_db import QdrantVectorDB
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot",
    description="A RAG system for the Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class QuestionRequest(BaseModel):
    question: str
    context: Optional[str] = None  # User-selected text context
    max_tokens: Optional[int] = 500

class ChatResponse(BaseModel):
    answer: str
    sources: List[str]
    confidence: float

class DocumentChunk(BaseModel):
    content: str
    metadata: dict

# Initialize vector database
vector_db = QdrantVectorDB()

# Load book contents from the actual documentation files
def load_book_contents():
    """Load book contents from the documentation files"""
    book_contents = []
    docs_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'docs')

    # Find all markdown files in the docs directory
    md_files = glob.glob(os.path.join(docs_dir, "*.md"))

    for file_path in md_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

                # Extract title from the markdown frontmatter or first heading
                lines = content.split('\n')
                title = "Untitled"
                for line in lines:
                    if line.strip().startswith('# '):
                        title = line.strip()[2:]  # Remove '# ' prefix
                        break
                    elif line.strip().startswith('title:'):
                        title = line.strip()[7:]  # Remove 'title:' prefix
                        break

                # Create document entry
                doc_id = os.path.basename(file_path).replace('.md', '')
                book_contents.append({
                    "id": doc_id,
                    "title": title,
                    "content": content
                })
        except Exception as e:
            print(f"Error reading file {file_path}: {e}")

    return book_contents

# Load book contents and add to vector DB
def initialize_vector_db():
    """Initialize the vector database with book content"""
    book_contents = load_book_contents()

    # Split contents into chunks for better retrieval
    chunks = []
    for doc in book_contents:
        # Split content into smaller chunks (sentences or paragraphs)
        content = doc['content']
        # Remove markdown headers and metadata from content for processing
        lines = content.split('\n')
        clean_content = '\n'.join(line for line in lines if not line.startswith('---') and not line.strip() == '')

        # Split into paragraphs
        paragraphs = clean_content.split('\n\n')
        for i, paragraph in enumerate(paragraphs):
            if len(paragraph.strip()) > 50:  # Only include substantial paragraphs
                chunks.append({
                    'content': paragraph.strip(),
                    'document_id': doc['id'],
                    'title': doc['title'],
                    'source': doc['title']
                })

    # Add chunks to vector database
    vector_db.add_document_chunks(chunks)
    print(f"Added {len(chunks)} chunks to vector database from {len(book_contents)} documents")

# Initialize the database when the app starts
initialize_vector_db()

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics RAG Chatbot API"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: QuestionRequest):
    """
    Main chat endpoint that processes user questions and returns answers
    based on the book content using RAG.
    """
    try:
        # If user provides context, use that primarily
        if request.context and len(request.context.strip()) > 10:
            # Use the user-provided context as the primary source
            relevant_chunks = [{
                'content': request.context,
                'metadata': {'source': 'user_selected_text'},
                'score': 1.0
            }]
        else:
            # Search the vector database for relevant chunks
            relevant_chunks = vector_db.search(request.question, top_k=5)

        if not relevant_chunks:
            return ChatResponse(
                answer="I couldn't find any relevant information in the Physical AI & Humanoid Robotics book to answer your question. Please try rephrasing your question or check if the topic is covered in the book.",
                sources=[],
                confidence=0.0
            )

        # Construct context from retrieved chunks
        context_text = "\n\n".join([chunk['content'] for chunk in relevant_chunks])
        sources = list(set([chunk['metadata']['source'] for chunk in relevant_chunks if chunk['metadata']['source'] != 'user_selected_text']))

        # In a real implementation, you would call an LLM here to generate the answer
        # For this implementation, we'll return a formatted response based on retrieved content
        answer = f"Based on the Physical AI & Humanoid Robotics book:\n\n{context_text}\n\nFor more details, see the relevant sections in the book."

        # Calculate confidence based on number of relevant chunks and their scores
        avg_score = sum([chunk.get('score', 0) for chunk in relevant_chunks]) / len(relevant_chunks)
        confidence = min(avg_score / 2.0, 1.0)  # Normalize to 0-1 range

        return ChatResponse(
            answer=answer,
            sources=sources,
            confidence=confidence
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing question: {str(e)}")

@app.get("/documents")
async def get_documents():
    """Return list of available documents"""
    book_contents = load_book_contents()
    return {"documents": book_contents}

@app.get("/search")
async def search_documents(query: str = Query(..., min_length=1, max_length=500)):
    """Search for relevant content in the book"""
    results = vector_db.search(query, top_k=10)
    return {"results": results}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)