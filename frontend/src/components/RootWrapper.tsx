import React from 'react';
import TextSelector from './TextSelector';
import FloatingChatbot from './FloatingChatbot';

interface RootWrapperProps {
  children: React.ReactNode;
}

const RootWrapper: React.FC<RootWrapperProps> = ({ children }) => {
  return (
    <TextSelector>
      {children}
      <FloatingChatbot />
    </TextSelector>
  );
};

export default RootWrapper;