import React, { useState, useEffect, useRef } from 'react';

interface LiveCodeProps {
  code: string;
  language?: string;
  title?: string;
}

const LiveCode: React.FC<LiveCodeProps> = ({ code, language = 'python', title = 'Code Editor' }) => {
  const [codeValue, setCodeValue] = useState(code);
  const [output, setOutput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [executionTime, setExecutionTime] = useState<number | null>(null);
  const [error, setError] = useState('');
  const codeRef = useRef<HTMLTextAreaElement>(null);

  // Initialize Thebe when component mounts
  useEffect(() => {
    const initThebe = async () => {
      if (typeof window !== 'undefined' && (window as any).thebelab) {
        try {
          await (window as any).thebelab.bootstrap();
        } catch (err) {
          console.log('Thebe not available or failed to initialize');
        }
      }
    };

    // Check if Thebe is already loaded or load it
    if (typeof window !== 'undefined') {
      if (!(window as any).thebelab) {
        // Load Thebe script dynamically
        const script = document.createElement('script');
        script.src = '/thebe/dist/thebe.js';
        script.async = true;
        script.onload = () => {
          initThebe();
        };
        document.head.appendChild(script);
      } else {
        initThebe();
      }
    }
  }, []);

  const handleRunCode = async () => {
    setIsLoading(true);
    setError('');
    setOutput('');
    setExecutionTime(null);

    try {
      // Record start time
      const startTime = performance.now();

      // In a real implementation, this would connect to a code execution service
      // For now, we'll simulate the execution
      await new Promise(resolve => setTimeout(resolve, 500));

      // Calculate execution time
      const endTime = performance.now();
      setExecutionTime(endTime - startTime);

      // For demonstration purposes, we'll just show the code as output
      // In a real implementation, this would execute the code and return the result
      setOutput(`Code executed successfully in ${(endTime - startTime).toFixed(2)}ms\n\nResult would appear here in a real implementation.`);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during execution');
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    setCodeValue(code); // Reset to original code
    setOutput('');
    setError('');
    setExecutionTime(null);
  };

  return (
    <div className="live-code-container" style={{
      border: '1px solid #00D4FF',
      borderRadius: '8px',
      padding: '16px',
      margin: '16px 0',
      backgroundColor: '#0a0a0a',
      fontFamily: 'monospace'
    }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '12px' }}>
        <h4 style={{ color: '#00D4FF', margin: 0 }}>{title}</h4>
        <div style={{ display: 'flex', gap: '8px' }}>
          <button
            onClick={handleRunCode}
            disabled={isLoading}
            style={{
              backgroundColor: '#00D4FF',
              color: '#000',
              border: 'none',
              padding: '6px 12px',
              borderRadius: '4px',
              cursor: isLoading ? 'not-allowed' : 'pointer',
              fontWeight: 'bold'
            }}
          >
            {isLoading ? 'Running...' : 'Run'}
          </button>
          <button
            onClick={handleReset}
            style={{
              backgroundColor: '#333',
              color: 'white',
              border: 'none',
              padding: '6px 12px',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Reset
          </button>
        </div>
      </div>

      <textarea
        ref={codeRef}
        value={codeValue}
        onChange={(e) => setCodeValue(e.target.value)}
        style={{
          width: '100%',
          minHeight: '150px',
          padding: '12px',
          fontFamily: 'monospace',
          fontSize: '14px',
          backgroundColor: '#000',
          color: '#e0e0e0',
          border: '1px solid #333',
          borderRadius: '4px',
          marginBottom: '12px'
        }}
        spellCheck={false}
      />

      {(output || error) && (
        <div
          className="live-code-output"
          style={{
            padding: '12px',
            backgroundColor: error ? '#440000' : '#001a00',
            border: `1px solid ${error ? '#880000' : '#008800'}`,
            borderRadius: '4px',
            fontFamily: 'monospace',
            fontSize: '14px',
            color: error ? '#ff6666' : '#66ff66',
            whiteSpace: 'pre-wrap',
            maxHeight: '200px',
            overflowY: 'auto'
          }}
        >
          {error ? `Error: ${error}` : output}
          {executionTime !== null && !error && (
            <div style={{ marginTop: '8px', fontSize: '12px', color: '#ccc' }}>
              Execution time: {executionTime.toFixed(2)}ms
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default LiveCode;