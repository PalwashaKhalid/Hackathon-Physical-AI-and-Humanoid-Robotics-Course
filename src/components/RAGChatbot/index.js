import React from 'react';
import RAGChatbot from './RAGChatbot';

// Export the RAGChatbot component as default
export default function RAGChatbotWrapper(props) {
  return <RAGChatbot {...props} />;
}