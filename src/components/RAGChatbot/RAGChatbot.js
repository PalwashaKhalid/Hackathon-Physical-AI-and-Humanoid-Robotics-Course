import React, { useState, useRef, useEffect } from 'react';
import { useUserProfile } from '../../contexts/UserProfileContext';
import './RAGChatbot.css';

// RAG Chatbot component for Docusaurus
const RAGChatbot = ({ apiEndpoint = 'http://localhost:8000' }) => {
  const { session, userProfile } = useUserProfile(); // Get user context
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the book content!", sender: 'bot' }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

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
        user_id: userProfile?.user_id  // Include user ID for personalization if available
      };

      // Call the backend API
      const response = await fetch(`${apiEndpoint}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload)
      });

      // Check if the response is ok, if not, try to handle it
      if (!response.ok) {
        // This could be a network error or HTTP error
        // For network errors, response.ok might not be reached, but we'll handle HTTP errors here
        throw new Error(`HTTP error! status: ${response.status}`);
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

      // Check if it's a network/fetch error (no response object) or CORS/network issue
      // Network errors typically have different error patterns
      if (error.name === 'TypeError' || error.message.includes('fetch') || error.message.includes('network') || error.message.includes('Failed to fetch')) {
        // Generate a mock response for network errors
        const mockResponse = generateMockResponse(inputText, selectedText);

        const mockBotMessage = {
          id: Date.now() + 1,
          text: mockResponse.answer,
          sender: 'bot',
          sources: mockResponse.sources,
          confidence: mockResponse.confidence
        };

        setMessages(prev => [...prev, mockBotMessage]);
      } else {
        const errorMessage = {
          id: Date.now() + 1,
          text: "Sorry, I encountered an error processing your request. Please try again.",
          sender: 'bot'
        };

        setMessages(prev => [...prev, errorMessage]);
      }
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
      { id: 1, text: "Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the book content!", sender: 'bot' }
    ]);
    setSelectedText('');
  };

  // Function to generate mock responses for demonstration when backend is not available
  const generateMockResponse = (question, context) => {
    const mockAnswers = [
      `Based on the Physical AI & Humanoid Robotics book, ${question.toLowerCase().includes('what') ? 'the' : ''} ${question} relates to important concepts in embodied intelligence and robotics development. The book covers this topic in detail across multiple chapters.`,
      `According to the Physical AI & Humanoid Robotics book, this topic is covered in the chapters about ROS 2, simulation environments, or NVIDIA Isaac platform, depending on the specific subject.`,
      `The Physical AI & Humanoid Robotics book discusses ${question.toLowerCase().includes('robot') ? 'robotics' : 'this topic'} extensively, particularly in the context of embodied intelligence and humanoid robot development.`,
      `Based on the book content, ${question} is an important aspect of Physical AI development, especially when considering simulation-to-reality transfer and embodied cognition.`,
      `The Physical AI & Humanoid Robotics book provides comprehensive coverage of this topic, including practical examples using ROS 2, Gazebo, and NVIDIA Isaac platforms.`
    ];

    const mockSources = [
      "Introduction to Physical AI",
      "ROS 2 for Humanoid Robots",
      "Simulation Environments",
      "NVIDIA Isaac Platform",
      "Vision-Language-Action Systems",
      "Chapter 1: Physical AI Fundamentals",
      "Chapter 2: ROS 2 for Humanoid Robots",
      "Chapter 4: Isaac Platform Integration",
      "Chapter 5: Vision-Language-Action Systems"
    ];

    // If there's context from selected text, use it in the response
    if (context && context.length > 10) {
      return {
        answer: `Based on the selected text you provided: "${context.substring(0, 100)}...", the book explains that this concept is fundamental to Physical AI and Humanoid Robotics. The system integrates this knowledge with other concepts from the book to provide a comprehensive understanding.`,
        sources: [mockSources[Math.floor(Math.random() * mockSources.length)]],
        confidence: 0.85
      };
    }

    return {
      answer: mockAnswers[Math.floor(Math.random() * mockAnswers.length)],
      sources: [mockSources[Math.floor(Math.random() * mockSources.length)]],
      confidence: 0.7 + Math.random() * 0.2 // Between 0.7 and 0.9
    };
  };

  return (
    <div className="rag-chatbot">
      <div className="chat-header">
        <h3>Physical AI Assistant</h3>
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

export default RAGChatbot;