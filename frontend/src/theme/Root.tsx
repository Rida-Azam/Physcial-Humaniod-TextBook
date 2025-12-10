import React from 'react';
import RootWrapper from '../components/RootWrapper';
import { ChatProvider } from '../context/ChatContext';

interface DocusaurusRootProps {
  children: React.ReactNode;
}

const Root: React.FC<DocusaurusRootProps> = ({ children }) => {
  return (
    <ChatProvider>
      <RootWrapper>{children}</RootWrapper>
    </ChatProvider>
  );
};

export default Root;