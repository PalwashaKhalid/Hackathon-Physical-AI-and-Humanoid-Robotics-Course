import React, { useState, useEffect } from 'react';
import { useUserProfile } from '../contexts/UserProfileContext';
import '../components/RAGChatbot/RAGChatbot.css';

// Separate component for the authenticated chatbot
const RAGChatbotWithAuth = () => {
  const { session, userProfile } = useUserProfile();
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your personalized Physical AI & Humanoid Robotics assistant. How can I help you today?", sender: 'bot' }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = React.useRef(null);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText.length > 0) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Function to send message to the API
  const sendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Prepare the request payload with user context
      const payload = {
        question: inputText,
        context: selectedText || null,  // Include selected text context if available
        user_id: userProfile?.user_id  // Include user ID for personalization
      };

      // Call the backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to the chat
      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: data.sources,
        confidence: data.confidence
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error processing your request. Please try again.",
        sender: 'bot'
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  // Handle Enter key press
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Function to clear chat
  const clearChat = () => {
    setMessages([
      { id: 1, text: "Hello! I'm your personalized Physical AI & Humanoid Robotics assistant. How can I help you today?", sender: 'bot' }
    ]);
    setSelectedText('');
  };

  return (
    <div className="rag-chatbot">
      <div className="chat-header">
        <h3>Personalized AI Assistant</h3>
        <button onClick={clearChat} className="clear-btn">Clear Chat</button>
      </div>

      <div className="chat-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.sender}-message`}
          >
            <div className="message-content">
              <p>{message.text}</p>
              {message.sources && message.sources.length > 0 && (
                <div className="sources">
                  <small>Sources: {message.sources.join(', ')}</small>
                </div>
              )}
              {message.confidence !== undefined && (
                <div className="confidence">
                  <small>Confidence: {(message.confidence * 100).toFixed(1)}%</small>
                </div>
              )}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <p className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </p>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {selectedText && (
        <div className="selected-text-notice">
          <strong>Using selected text as context:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
        </div>
      )}

      <div className="chat-input-area">
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about Physical AI & Humanoid Robotics..."
          rows="3"
          disabled={isLoading}
        />
        <button
          onClick={sendMessage}
          disabled={!inputText.trim() || isLoading}
          className="send-btn"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>

      <div className="chat-footer">
        <small>Select text on the page to ask specific questions about it</small>
      </div>
    </div>
  );
};

const Dashboard = () => {
  const { session, userProfile, loading } = useUserProfile();
  const [personalizedContent, setPersonalizedContent] = useState(null);

  useEffect(() => {
    const fetchPersonalizedContent = async () => {
      if (userProfile?.user_id) {
        try {
          const response = await fetch(`/api/auth/personalized-content/${userProfile.user_id}`);
          if (response.ok) {
            const data = await response.json();
            setPersonalizedContent(data);
          }
        } catch (error) {
          console.error('Error fetching personalized content:', error);
        }
      }
    };

    if (userProfile) {
      fetchPersonalizedContent();
    }
  }, [userProfile]);

  if (loading) {
    return <div className="dashboard-container">Loading...</div>;
  }

  if (!session) {
    return (
      <div className="dashboard-container">
        <h1>Access Denied</h1>
        <p>Please sign in to access the dashboard.</p>
      </div>
    );
  }

  return (
    <div className="dashboard-container">
      <div className="dashboard-header">
        <h1>Welcome back, {session.user?.name || 'User'}!</h1>
        <p>Here's your personalized learning experience based on your profile.</p>
      </div>

      {userProfile && (
        <div className="user-profile-summary">
          <h2>Your Profile</h2>
          <div className="profile-info">
            <p><strong>Experience Level:</strong> {userProfile.experience_level || 'Not specified'}</p>
            <p><strong>Role:</strong> {userProfile.role || 'Not specified'}</p>
            <p><strong>Interests:</strong> {userProfile.robotics_interest?.join(', ') || 'Not specified'}</p>
            <p><strong>Learning Goal:</strong> {userProfile.learning_goal || 'Not specified'}</p>
          </div>
        </div>
      )}

      {personalizedContent && personalizedContent.recommendations.length > 0 && (
        <div className="recommendations-section">
          <h2>Recommended for You</h2>
          <div className="recommendations-list">
            {personalizedContent.recommendations.map((rec, index) => (
              <div key={index} className="recommendation-card">
                <h3>{rec.title}</h3>
                <p>{rec.description}</p>
                <a href={rec.link} className="recommendation-link">Explore Content</a>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className="dashboard-chatbot">
        <h2>Your Personalized Assistant</h2>
        <div style={{ height: '500px' }}>
          <RAGChatbotWithAuth />
        </div>
      </div>
    </div>
  );
};

export default Dashboard;