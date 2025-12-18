import React from 'react';
import { UserProfileProvider } from '../../contexts/UserProfileContext';

// Root component that wraps the entire app with authentication context
export default function Root({ children }) {
  return (
    <UserProfileProvider>
      {children}
    </UserProfileProvider>
  );
}