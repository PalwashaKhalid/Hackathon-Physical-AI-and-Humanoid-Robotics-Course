---
sidebar_position: 0
---

# RAG Chatbot Integration Guide

This guide explains how to use the integrated RAG (Retrieval-Augmented Generation) chatbot in the Physical AI & Humanoid Robotics book.

## How It Works

The RAG chatbot uses advanced AI techniques to answer questions about the book content:

1. **Retrieval**: When you ask a question, the system searches through the book content to find relevant passages
2. **Augmentation**: The most relevant content is used to inform the response
3. **Generation**: An AI model generates a contextual answer based on the retrieved information

## Using the Chatbot

### Asking General Questions

Simply type your question in the chat interface and press Enter or click Send. The chatbot will search the book content and provide an answer with sources.

### Using Selected Text Context

1. **Select text** on any page of the book by highlighting it with your mouse
2. The chatbot will detect the selected text and show a notice
3. Ask your question - the system will prioritize the selected text in its response

This is particularly useful when you want to ask detailed questions about specific concepts or ask for clarifications on complex topics.

## Features

- **Source Attribution**: Responses include sources from the book
- **Confidence Scores**: Each response includes a confidence level
- **Context Awareness**: Uses selected text for more targeted answers
- **Real-time Interaction**: Instant responses to your questions

## Example Use Cases

- "Explain the concept of embodied intelligence"
- "What are the key components of ROS 2 for humanoid robots?"
- "How does simulation help in humanoid robotics development?"
- "What is the difference between Gazebo and Isaac Sim?"

## Technical Implementation

The RAG system is built using:

- **FastAPI**: Backend API server
- **Qdrant**: Vector database for document storage and similarity search
- **Sentence Transformers**: For creating text embeddings
- **OpenAI API**: For response generation (in production setup)

The system indexes all book content and creates vector embeddings that allow for semantic search, meaning it can find relevant content even if your question uses different terminology than the book.

import RAGChatbot from '@site/src/components/RAGChatbot';

<div className="rag-chatbot-container">
  <h2>Try the Chatbot</h2>
  <p>Ask any question about Physical AI & Humanoid Robotics!</p>
  <div style={{height: '500px', margin: '20px 0'}}>
    <RAGChatbot apiEndpoint="http://localhost:8000" />
  </div>
</div>