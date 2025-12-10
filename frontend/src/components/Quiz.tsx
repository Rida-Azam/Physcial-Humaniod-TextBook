import React, { useState } from 'react';

interface QuizProps {
  question: string;
  options: string[];
  correctAnswer: number;
  explanation?: string;
}

const Quiz: React.FC<QuizProps> = ({ question, options, correctAnswer, explanation }) => {
  const [selectedOption, setSelectedOption] = useState<number | null>(null);
  const [submitted, setSubmitted] = useState(false);
  const [feedback, setFeedback] = useState('');

  const handleSubmit = () => {
    if (selectedOption === null) {
      setFeedback('Please select an answer.');
      return;
    }

    if (selectedOption === correctAnswer) {
      setFeedback('Correct! Well done.');
    } else {
      setFeedback('Incorrect. Try again!');
    }
    setSubmitted(true);
  };

  const handleReset = () => {
    setSelectedOption(null);
    setSubmitted(false);
    setFeedback('');
  };

  return (
    <div className="quiz-container" style={{
      border: '1px solid #00D4FF',
      borderRadius: '8px',
      padding: '16px',
      margin: '16px 0',
      backgroundColor: '#0f0f0f',
      color: 'white'
    }}>
      <h4 style={{ color: '#00D4FF' }}>{question}</h4>
      <div style={{ margin: '12px 0' }}>
        {options.map((option, index) => (
          <div key={index} style={{ margin: '8px 0' }}>
            <label style={{ display: 'flex', alignItems: 'center' }}>
              <input
                type="radio"
                name={`quiz-${question}`}
                checked={selectedOption === index}
                onChange={() => {
                  setSelectedOption(index);
                  if (submitted) {
                    setSubmitted(false);
                    setFeedback('');
                  }
                }}
                disabled={submitted}
                style={{ marginRight: '8px' }}
              />
              {option}
            </label>
          </div>
        ))}
      </div>

      {!submitted ? (
        <button
          onClick={handleSubmit}
          style={{
            backgroundColor: '#00D4FF',
            color: '#000',
            border: 'none',
            padding: '8px 16px',
            borderRadius: '4px',
            cursor: 'pointer',
            fontWeight: 'bold'
          }}
        >
          Submit
        </button>
      ) : (
        <div style={{ marginTop: '12px' }}>
          <div style={{
            padding: '8px',
            borderRadius: '4px',
            backgroundColor: selectedOption === correctAnswer ? '#006400' : '#8B0000',
            color: 'white',
            marginBottom: '8px'
          }}>
            {feedback}
          </div>
          {explanation && selectedOption === correctAnswer && (
            <div style={{ fontStyle: 'italic', color: '#ccc' }}>
              {explanation}
            </div>
          )}
          <button
            onClick={handleReset}
            style={{
              backgroundColor: '#333',
              color: 'white',
              border: 'none',
              padding: '6px 12px',
              borderRadius: '4px',
              cursor: 'pointer',
              marginTop: '8px'
            }}
          >
            Try Again
          </button>
        </div>
      )}
    </div>
  );
};

export default Quiz;