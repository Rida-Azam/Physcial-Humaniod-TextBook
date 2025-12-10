import React, { useState } from 'react';

interface AccordionItem {
  title: string;
  content: string | React.ReactNode;
}

interface AccordionProps {
  items: AccordionItem[];
  multiple?: boolean;
}

const Accordion: React.FC<AccordionProps> = ({ items, multiple = false }) => {
  const [activeIndexes, setActiveIndexes] = useState<number[]>([]);

  const toggleItem = (index: number) => {
    if (multiple) {
      if (activeIndexes.includes(index)) {
        setActiveIndexes(activeIndexes.filter(i => i !== index));
      } else {
        setActiveIndexes([...activeIndexes, index]);
      }
    } else {
      if (activeIndexes.includes(index)) {
        setActiveIndexes([]);
      } else {
        setActiveIndexes([index]);
      }
    }
  };

  return (
    <div className="accordion-container" style={{ margin: '16px 0' }}>
      {items.map((item, index) => (
        <div
          key={index}
          className="accordion-item"
          style={{
            border: '1px solid #333',
            borderRadius: '4px',
            marginBottom: '8px',
            overflow: 'hidden'
          }}
        >
          <button
            onClick={() => toggleItem(index)}
            style={{
              width: '100%',
              padding: '12px 16px',
              textAlign: 'left',
              background: '#0f0f0f',
              color: '#00D4FF',
              border: 'none',
              cursor: 'pointer',
              fontWeight: 'bold',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            {item.title}
            <span style={{ fontSize: '1.2em' }}>
              {activeIndexes.includes(index) ? '−' : '+'}
            </span>
          </button>
          {activeIndexes.includes(index) && (
            <div
              className="accordion-content"
              style={{
                padding: '16px',
                backgroundColor: '#0a0a0a',
                color: '#e0e0e0',
                borderTop: '1px solid #333'
              }}
            >
              {typeof item.content === 'string' ? (
                <div dangerouslySetInnerHTML={{ __html: item.content }} />
              ) : (
                item.content
              )}
            </div>
          )}
        </div>
      ))}
    </div>
  );
};

export default Accordion;