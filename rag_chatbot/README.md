# Physical AI & Humanoid Robotics RAG Chatbot

This RAG (Retrieval-Augmented Generation) chatbot is integrated into the Physical AI & Humanoid Robotics book to provide interactive learning assistance. The system allows users to ask questions about the book content and get AI-powered answers based on the text.

## Features

- **Semantic Search**: Uses vector embeddings to find relevant content
- **Context Awareness**: Can answer questions based on selected text
- **Source Attribution**: Shows which documents the answers come from
- **Confidence Scoring**: Provides confidence levels for answers
- **Interactive UI**: Embedded chat interface in the book

## Architecture

The system uses:
- **FastAPI**: Backend API server
- **Qdrant**: Vector database for document storage and similarity search
- **Sentence Transformers**: For creating text embeddings (free model)
- **React**: Frontend chat interface

## Setup

### Prerequisites

- Python 3.8+
- Node.js (for Docusaurus site)

### Installation

1. Install Python dependencies:
```bash
cd rag_chatbot
pip install -r requirements.txt
```

2. Set up environment variables:
```bash
cp .env.example .env
# Edit .env to add your configuration
```

For local development, you can use the default settings which use in-memory Qdrant.

### Running the Backend

```bash
cd rag_chatbot
python start_api.py
```

The API will be available at `http://localhost:8000`

### Running the Frontend (Docusaurus)

```bash
# From the main project directory
npm install
npm run start
```

## Configuration

### Local Development (Default)

By default, the system uses in-memory Qdrant for local development. Set in `.env`:
```
LOCAL_QDRANT=true
```

### Qdrant Cloud (Production)

To use Qdrant Cloud Free Tier:

1. Sign up at [Qdrant Cloud](https://qdrant.tech/)
2. Create a cluster
3. Update `.env`:
```
LOCAL_QDRANT=false
QDRANT_URL=https://your-cluster-url.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_api_key_here
```

### Neon Postgres (Optional)

While not directly used in this implementation, the architecture supports Neon Postgres for metadata storage:

```
NEON_DB_URL=postgresql://username:password@ep-xxx.us-east-4.aws.neon.tech/dbname?sslmode=require
```

## API Endpoints

- `GET /` - Health check
- `POST /chat` - Chat endpoint for asking questions
- `GET /documents` - List available documents
- `GET /search` - Search documents by query

### Chat Request Format

```json
{
  "question": "Your question here",
  "context": "Optional selected text context",
  "max_tokens": 500
}
```

### Chat Response Format

```json
{
  "answer": "Generated answer",
  "sources": ["document titles"],
  "confidence": 0.85
}
```

## Integration with Docusaurus

The chatbot is integrated into the Docusaurus site using a React component. To add the chatbot to any page, use:

```md
import RAGChatbot from '@site/src/components/RAGChatbot';

<RAGChatbot />
```

## Using the Chatbot

1. **General Questions**: Type any question about Physical AI or Humanoid Robotics
2. **Selected Text**: Highlight text on the page, and the chatbot will use it as context
3. **Source Tracking**: Answers include references to the book sections they're based on

## Development

The system is designed to be extensible:

- Add new document sources by updating the document loading function
- Integrate different LLMs for response generation
- Enhance the frontend with additional features

## Free Tier Usage

This implementation is designed to work within free tier limits:

- Qdrant Cloud Free Tier: 100k vectors, 1M requests/month
- Uses free sentence transformer model
- Optional OpenAI API for enhanced responses (not required for basic functionality)

## Deployment

To deploy the backend API separately (required for production):

### Deploy to Render (Recommended - Free Tier)
1. Fork this repository to your GitHub account
2. Go to [Render Dashboard](https://dashboard.render.com)
3. Click "New +" → "Web Service"
4. Connect to your GitHub account and select your forked repository
5. Configure the service:
   - **Environment**: Python
   - **Build Root**: `rag_chatbot`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn api.main:app --host 0.0.0.0 --port $PORT`
6. **Environment Variables**:
   - `LOCAL_QDRANT=true` (Uses in-memory storage for free tier)
7. Click "Create Web Service"

### Deploy to Railway
1. Install Railway CLI or use the web interface
2. Create a new project and link to your repository
3. Set the following environment variables:
   - `LOCAL_QDRANT=true` (for in-memory storage) or configure Qdrant Cloud
4. Set the start command to: `cd rag_chatbot && python start_api.py`

### Using the Deployed Backend
Once deployed, update the frontend to use your backend URL:
```md
import RAGChatbot from '@site/src/components/RAGChatbot';

<RAGChatbot apiEndpoint="https://your-deployed-backend-url.onrender.com" />
```

For complete deployment instructions, see [DEPLOYMENT.md](DEPLOYMENT.md).

## Troubleshooting

If you encounter issues:

1. Verify the backend is running: `http://localhost:8000`
2. Check browser console for frontend errors
3. Ensure document files are properly loaded from the `/docs` directory
4. Confirm environment variables are set correctly
5. For deployed sites, ensure the backend API is accessible and CORS is configured