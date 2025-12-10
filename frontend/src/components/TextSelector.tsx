import React, { useState, useEffect, useRef } from 'react';
import { useChat } from '../context/ChatContext';

interface TextSelectorProps {
  children: React.ReactNode;
}

const TextSelector: React.FC<TextSelectorProps> = ({ children }) => {
  const [selection, setSelection] = useState<{
    text: string;
    rect: DOMRect;
  } | null>(null);
  const [showPopup, setShowPopup] = useState(false);
  const popupRef = useRef<HTMLDivElement>(null);
  const { sendMessage } = useChat(); // Assuming we have a chat context

  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText) {
        const sel = window.getSelection();
        if (sel && sel.rangeCount > 0) {
          const range = sel.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          setSelection({
            text: selectedText,
            rect: rect
          });
          setShowPopup(true);
        }
      } else {
        setShowPopup(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (popupRef.current && !popupRef.current.contains(event.target as Node)) {
        setShowPopup(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const handleExplain = () => {
    if (selection) {
      // Send explanation request to the RAG system
      sendMessage(`Explain this text: "${selection.text}" in the context of Physical AI and Humanoid Robotics`);
      setShowPopup(false);
    }
  };

  const handleAsk = () => {
    if (selection) {
      // Open chatbot with selected text as context
      sendMessage(`I selected this text: "${selection.text}". Can you explain it or answer questions about it?`);
      setShowPopup(false);
    }
  };

  return (
    <>
      {children}
      {showPopup && selection && (
        <div
          ref={popupRef}
          className="text-selection-popup"
          style={{
            position: 'fixed',
            top: selection.rect.top - 40,
            left: selection.rect.left,
            zIndex: 1000,
            backgroundColor: '#00D4FF',
            color: '#000',
            padding: '8px 12px',
            borderRadius: '4px',
            boxShadow: '0 2px 10px rgba(0,0,0,0.2)',
            fontSize: '14px',
            display: 'flex',
            gap: '8px',
          }}
        >
          <button
            onClick={handleExplain}
            style={{
              background: 'none',
              border: 'none',
              padding: '4px 8px',
              cursor: 'pointer',
              fontWeight: 'bold',
            }}
          >
            Explain
          </button>
          <button
            onClick={handleAsk}
            style={{
              background: 'none',
              border: 'none',
              padding: '4px 8px',
              cursor: 'pointer',
              fontWeight: 'bold',
            }}
          >
            Ask
          </button>
        </div>
      )}
    </>
  );
};

export default TextSelector;