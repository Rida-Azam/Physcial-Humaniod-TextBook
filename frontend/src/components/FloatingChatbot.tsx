import React, { useState } from 'react';
import Chatbot from './Chatbot';

const FloatingChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isChatbotOpen, setIsChatbotOpen] = useState(false);

  const toggleChat = () => {
    if (!isOpen) {
      setIsOpen(true);
      setTimeout(() => setIsChatbotOpen(true), 10);
    } else {
      setIsChatbotOpen(false);
      setTimeout(() => setIsOpen(false), 300);
    }
  };

  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      zIndex: 1000,
      fontFamily: 'sans-serif'
    }}>
      {isOpen && (
        <div
          style={{
            marginBottom: '10px',
            transform: isOpen ? 'translateY(0)' : 'translateY(20px)',
            opacity: isOpen ? '1' : '0',
            transition: 'transform 0.3s ease, opacity 0.3s ease'
          }}
        >
          {isChatbotOpen && (
            <div style={{
              width: '300px',
              height: '400px',
              backgroundColor: 'white',
              borderRadius: '8px',
              boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
              display: 'flex',
              flexDirection: 'column',
              overflow: 'hidden'
            }}>
              <div style={{
                backgroundColor: '#00D4FF',
                color: '#000',
                padding: '10px',
                fontWeight: 'bold',
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center'
              }}>
                <span>Textbook AI Assistant</span>
                <button
                  onClick={toggleChat}
                  style={{
                    background: 'none',
                    border: 'none',
                    fontSize: '18px',
                    cursor: 'pointer',
                    color: '#000',
                    fontWeight: 'bold'
                  }}
                >
                  ×
                </button>
              </div>
              <div style={{
                flex: 1,
                overflow: 'auto',
                padding: '10px'
              }}>
                <Chatbot />
              </div>
            </div>
          )}
        </div>
      )}

      <button
        onClick={toggleChat}
        style={{
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#00D4FF',
          color: '#000',
          border: 'none',
          fontSize: '24px',
          fontWeight: 'bold',
          cursor: 'pointer',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          boxShadow: '0 4px 8px rgba(0,0,0,0.2)',
          transition: 'transform 0.2s ease, box-shadow 0.2s ease'
        }}
        onMouseEnter={(e) => {
          (e.target as HTMLElement).style.transform = 'scale(1.1)';
          (e.target as HTMLElement).style.boxShadow = '0 6px 12px rgba(0,0,0,0.3)';
        }}
        onMouseLeave={(e) => {
          (e.target as HTMLElement).style.transform = 'scale(1)';
          (e.target as HTMLElement).style.boxShadow = '0 4px 8px rgba(0,0,0,0.2)';
        }}
      >
        {isOpen ? '×' : '?'}
      </button>
    </div>
  );
};

export default FloatingChatbot;