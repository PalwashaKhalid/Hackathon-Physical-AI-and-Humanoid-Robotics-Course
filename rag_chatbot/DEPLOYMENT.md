# Deployment Guide for RAG Chatbot Backend

This guide explains how to deploy the RAG chatbot backend API to free cloud platforms.

## Deploy to Render (Recommended Free Option)

### Prerequisites
- A Render account (free tier available)

### Steps
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
   - `PORT=10000` (Render will provide this automatically)
7. Click "Create Web Service"

### After Deployment
- Render will provide a URL like `https://your-service-name.onrender.com`
- Update your Docusaurus site to use this URL:

```jsx
<RAGChatbot apiEndpoint="https://your-service-name.onrender.com" />
```

## Deploy to Railway

### Prerequisites
- A Railway account (free tier available)

### Steps
1. Install Railway CLI: `npm i -g @railway/cli`
2. Login: `railway login`
3. Navigate to the rag_chatbot directory: `cd rag_chatbot`
4. Link to a new project: `railway init`
5. Deploy: `railway up`

### Environment Variables
- `LOCAL_QDRANT=true`
- `PORT=8080`

## Deploy to Heroku (Alternative)

### Steps
1. Create a Heroku account
2. Install Heroku CLI
3. Create a new app in the Heroku dashboard
4. Connect to your GitHub repository
5. Enable automatic deploys
6. Set config vars:
   - `LOCAL_QDRANT=true`
   - `PORT=5000`

## Using the Deployed Backend

Once deployed, update your Docusaurus site:

1. In your `docs/chatbot-integration.md` file:
```md
import RAGChatbot from '@site/src/components/RAGChatbot';

<div className="rag-chatbot-container">
  <h2>Try the Chatbot</h2>
  <p>Ask any question about Physical AI & Humanoid Robotics!</p>
  <div style={{height: '500px', margin: '20px 0'}}>
    <RAGChatbot apiEndpoint="https://your-deployed-backend-url.onrender.com" />
  </div>
</div>
```

2. In your `docs/intro.md` file:
```md
<RAGChatbot apiEndpoint="https://your-deployed-backend-url.onrender.com" />
```

## Important Notes

- The free tier uses in-memory storage (LOCAL_QDRANT=true), which means data is reset when the service restarts
- For persistent storage, consider upgrading to Qdrant Cloud (has free tier) with 100k vectors and 1M requests/month
- The service may go to sleep on some platforms after inactivity - first requests might be slow

## Troubleshooting

If you encounter issues:
1. Check the deployment logs in your cloud platform dashboard
2. Verify environment variables are set correctly
3. Ensure CORS is properly configured (already done in the API)
4. Test the API endpoints directly in your browser: `https://your-deployed-url/`