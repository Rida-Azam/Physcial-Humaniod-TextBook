import React, { useState } from 'react';
import { useChat } from '../context/ChatContext';

const Chatbot: React.FC = () => {
  const { messages, sendMessage: sendContextMessage, isLoading } = useChat();
  const [inputValue, setInputValue] = useState('');

  const handleSendMessage = () => {
    if (!inputValue.trim() || isLoading) return;

    sendContextMessage(inputValue);
    setInputValue('');
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Textbook AI Assistant</h3>
      </div>
      <div className="chatbot-messages">
        {messages.map((message) => (
          <div
            key={message.id}
            className={`message ${message.role}`}
            style={{
              textAlign: message.role === 'user' ? 'right' : 'left',
              marginBottom: '10px'
            }}
          >
            <div
              style={{
                display: 'inline-block',
                padding: '8px 12px',
                borderRadius: '8px',
                backgroundColor: message.role === 'user' ? '#00D4FF' : '#f0f0f0',
                color: message.role === 'user' ? '#000' : '#000',
                maxWidth: '80%'
              }}
            >
              {message.content}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className="message assistant" style={{ textAlign: 'left', marginBottom: '10px' }}>
            <div
              style={{
                display: 'inline-block',
                padding: '8px 12px',
                borderRadius: '8px',
                backgroundColor: '#f0f0f0',
                color: 'black',
                maxWidth: '80%'
              }}
            >
              Thinking...
            </div>
          </div>
        )}
      </div>
      <div className="chatbot-input">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask a question about Physical AI & Humanoid Robotics..."
          rows={3}
          style={{
            width: '100%',
            padding: '8px',
            borderRadius: '4px',
            border: '1px solid #ccc'
          }}
        />
        <button
          onClick={handleSendMessage}
          disabled={isLoading || !inputValue.trim()}
          style={{
            marginTop: '8px',
            padding: '8px 16px',
            backgroundColor: '#00D4FF',
            color: '#000',
            border: 'none',
            borderRadius: '4px',
            cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer'
          }}
        >
          Send
        </button>
      </div>
    </div>
  );
};

export default Chatbot;