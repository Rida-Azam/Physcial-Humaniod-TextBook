import React from 'react';
import Quiz from '../components/Quiz';
import Accordion from '../components/Accordion';
import LiveCode from '../components/LiveCode';
import Chatbot from '../components/Chatbot';
import PersonalizeButton from '../components/PersonalizeButton';
import TranslateToggle from '../components/TranslateToggle';

// Default MDX components
import OriginalMDXComponents from '@theme-original/MDXComponents';

const MDXComponents = {
  // Keep all original components
  ...OriginalMDXComponents,
  // Add custom components
  Quiz,
  Accordion,
  LiveCode,
  Chatbot,
  PersonalizeButton,
  TranslateToggle,
};

export default MDXComponents;