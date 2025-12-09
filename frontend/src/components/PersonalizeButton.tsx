import React, { useState, useEffect } from 'react';

interface PersonalizationSettings {
  fontSize: 'small' | 'medium' | 'large';
  theme: 'light' | 'dark' | 'auto';
  readingLevel: 'beginner' | 'intermediate' | 'advanced';
  showHints: boolean;
  preferredLanguage: string;
}

const PersonalizeButton: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [settings, setSettings] = useState<PersonalizationSettings>({
    fontSize: 'medium',
    theme: 'auto',
    readingLevel: 'intermediate',
    showHints: true,
    preferredLanguage: 'en'
  });

  // Load saved settings from localStorage
  useEffect(() => {
    const savedSettings = localStorage.getItem('textbook-personalization');
    if (savedSettings) {
      try {
        setSettings(JSON.parse(savedSettings));
      } catch (e) {
        console.error('Error loading personalization settings', e);
      }
    }
  }, []);

  // Save settings to localStorage whenever they change
  useEffect(() => {
    localStorage.setItem('textbook-personalization', JSON.stringify(settings));

    // Apply theme setting
    if (settings.theme === 'dark') {
      document.documentElement.setAttribute('data-theme', 'dark');
    } else if (settings.theme === 'light') {
      document.documentElement.setAttribute('data-theme', 'light');
    } else {
      // Auto theme based on system preference
      const systemPrefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
      document.documentElement.setAttribute('data-theme', systemPrefersDark ? 'dark' : 'light');
    }

    // Apply font size setting
    document.documentElement.style.setProperty('--font-size-multiplier',
      settings.fontSize === 'small' ? '0.875' :
      settings.fontSize === 'large' ? '1.125' : '1'
    );
  }, [settings]);

  const handleSettingChange = (key: keyof PersonalizationSettings, value: any) => {
    setSettings(prev => ({
      ...prev,
      [key]: value
    }));
  };

  const resetSettings = () => {
    const defaultSettings: PersonalizationSettings = {
      fontSize: 'medium',
      theme: 'auto',
      readingLevel: 'intermediate',
      showHints: true,
      preferredLanguage: 'en'
    };
    setSettings(defaultSettings);
    localStorage.removeItem('textbook-personalization');
  };

  return (
    <div className="personalize-button-container">
      <button
        onClick={() => setIsOpen(!isOpen)}
        style={{
          padding: '8px 12px',
          backgroundColor: '#4a90e2',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer',
          fontSize: '14px'
        }}
      >
        🎛️ Personalize
      </button>

      {isOpen && (
        <div className="personalize-dropdown" style={{
          position: 'absolute',
          top: '100%',
          right: '0',
          backgroundColor: 'white',
          border: '1px solid #ccc',
          borderRadius: '4px',
          padding: '16px',
          boxShadow: '0 2px 10px rgba(0,0,0,0.1)',
          zIndex: 1000,
          minWidth: '300px'
        }}>
          <h4 style={{ margin: '0 0 12px 0', fontSize: '16px' }}>Personalization Settings</h4>

          <div style={{ marginBottom: '12px' }}>
            <label style={{ display: 'block', marginBottom: '4px' }}>Font Size:</label>
            <select
              value={settings.fontSize}
              onChange={(e) => handleSettingChange('fontSize', e.target.value)}
              style={{ width: '100%', padding: '4px' }}
            >
              <option value="small">Small</option>
              <option value="medium">Medium</option>
              <option value="large">Large</option>
            </select>
          </div>

          <div style={{ marginBottom: '12px' }}>
            <label style={{ display: 'block', marginBottom: '4px' }}>Theme:</label>
            <select
              value={settings.theme}
              onChange={(e) => handleSettingChange('theme', e.target.value)}
              style={{ width: '100%', padding: '4px' }}
            >
              <option value="light">Light</option>
              <option value="dark">Dark</option>
              <option value="auto">Auto (System)</option>
            </select>
          </div>

          <div style={{ marginBottom: '12px' }}>
            <label style={{ display: 'block', marginBottom: '4px' }}>Reading Level:</label>
            <select
              value={settings.readingLevel}
              onChange={(e) => handleSettingChange('readingLevel', e.target.value)}
              style={{ width: '100%', padding: '4px' }}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div style={{ marginBottom: '12px' }}>
            <label style={{ display: 'flex', alignItems: 'center' }}>
              <input
                type="checkbox"
                checked={settings.showHints}
                onChange={(e) => handleSettingChange('showHints', e.target.checked)}
                style={{ marginRight: '8px' }}
              />
              Show hints and explanations
            </label>
          </div>

          <div style={{ marginBottom: '16px' }}>
            <label style={{ display: 'block', marginBottom: '4px' }}>Preferred Language:</label>
            <select
              value={settings.preferredLanguage}
              onChange={(e) => handleSettingChange('preferredLanguage', e.target.value)}
              style={{ width: '100%', padding: '4px' }}
            >
              <option value="en">English</option>
              <option value="ur">Urdu</option>
              <option value="roman">Roman Urdu</option>
            </select>
          </div>

          <div style={{ display: 'flex', gap: '8px' }}>
            <button
              onClick={resetSettings}
              style={{
                flex: 1,
                padding: '6px',
                backgroundColor: '#f0f0f0',
                border: '1px solid #ccc',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              Reset
            </button>
            <button
              onClick={() => setIsOpen(false)}
              style={{
                flex: 1,
                padding: '6px',
                backgroundColor: '#4a90e2',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              Close
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;