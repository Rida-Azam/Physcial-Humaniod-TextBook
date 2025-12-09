import React, { useState, useEffect } from 'react';

interface TranslationToggleProps {
  text?: string;
  children?: React.ReactNode;
}

const TranslateToggle: React.FC<TranslationToggleProps> = ({ text, children }) => {
  const [currentLang, setCurrentLang] = useState<'en' | 'ur' | 'roman'>('en');
  const [translatedText, setTranslatedText] = useState<string>('');
  const [isLoading, setIsLoading] = useState(false);

  // Language options
  const languages = [
    { code: 'en', name: 'English', flag: '🇺🇸' },
    { code: 'ur', name: 'Urdu', flag: '🇵🇰' },
    { code: 'roman', name: 'Roman Urdu', flag: '🇵🇰' }
  ];

  // Get current language name
  const currentLanguage = languages.find(lang => lang.code === currentLang);

  // Handle translation when language changes
  useEffect(() => {
    const handleTranslation = async () => {
      if (currentLang === 'en' || !text) {
        setTranslatedText(text || '');
        return;
      }

      setIsLoading(true);
      try {
        // In a real implementation, this would call the backend translation API
        // const response = await fetch('/api/translate/translate', {
        //   method: 'POST',
        //   headers: { 'Content-Type': 'application/json' },
        //   body: JSON.stringify({
        //     text: text,
        //     source_lang: 'en',
        //     target_lang: currentLang
        //   })
        // });
        // const data = await response.json();
        // setTranslatedText(data.translated_text);

        // For now, show a placeholder
        if (currentLang === 'ur') {
          setTranslatedText(`[Urdu translation of: ${text?.substring(0, 20) || 'content'}...]`);
        } else {
          setTranslatedText(`[Roman Urdu translation of: ${text?.substring(0, 20) || 'content'}...]`);
        }
      } catch (error) {
        console.error('Translation error:', error);
        setTranslatedText(text || '');
      } finally {
        setIsLoading(false);
      }
    };

    handleTranslation();
  }, [currentLang, text]);

  const toggleLanguage = () => {
    const currentIndex = languages.findIndex(lang => lang.code === currentLang);
    const nextIndex = (currentIndex + 1) % languages.length;
    setCurrentLang(languages[nextIndex].code as 'en' | 'ur' | 'roman');
  };

  const switchToLanguage = (langCode: 'en' | 'ur' | 'roman') => {
    setCurrentLang(langCode);
  };

  return (
    <div className="translate-toggle-container">
      <div className="translate-controls" style={{ marginBottom: '12px' }}>
        <span style={{ marginRight: '8px' }}>🌐 Language:</span>
        <div style={{ display: 'inline-block', position: 'relative' }}>
          <button
            onClick={toggleLanguage}
            style={{
              padding: '4px 8px',
              backgroundColor: '#4a90e2',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '4px'
            }}
          >
            {currentLanguage?.flag} {currentLanguage?.name}
          </button>

          {/* Dropdown for language selection */}
          <div style={{
            position: 'absolute',
            top: '100%',
            left: '0',
            backgroundColor: 'white',
            border: '1px solid #ccc',
            borderRadius: '4px',
            boxShadow: '0 2px 10px rgba(0,0,0,0.1)',
            zIndex: 1000,
            minWidth: '120px'
          }}>
            {languages.map(lang => (
              <button
                key={lang.code}
                onClick={() => switchToLanguage(lang.code as 'en' | 'ur' | 'roman')}
                style={{
                  width: '100%',
                  padding: '6px 8px',
                  border: 'none',
                  background: currentLang === lang.code ? '#e6f0ff' : 'white',
                  textAlign: 'left',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  gap: '6px'
                }}
              >
                <span>{lang.flag}</span>
                <span>{lang.name}</span>
              </button>
            ))}
          </div>
        </div>
      </div>

      {text && (
        <div className="translated-content">
          {isLoading ? (
            <div style={{ fontStyle: 'italic', color: '#666' }}>Translating...</div>
          ) : (
            <div>{translatedText}</div>
          )}
        </div>
      )}

      {children && (
        <div className="original-content">
          {children}
        </div>
      )}
    </div>
  );
};

export default TranslateToggle;