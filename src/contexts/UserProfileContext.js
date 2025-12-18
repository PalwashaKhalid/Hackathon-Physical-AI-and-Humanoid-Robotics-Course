import React, { createContext, useContext, useEffect, useState } from 'react';
import { useSession } from '@better-auth/react';

const UserProfileContext = createContext();

export const UserProfileProvider = ({ children }) => {
  const { session, isPending } = useSession();
  const [userProfile, setUserProfile] = useState(null);
  const [loading, setLoading] = useState(true);

  // Load user profile when session changes
  useEffect(() => {
    const loadProfile = async () => {
      if (session?.user?.id) {
        try {
          const response = await fetch(`/api/auth/profile/${session.user.id}`, {
            headers: {
              'Authorization': `Bearer ${session.access_token}`
            }
          });
          if (response.ok) {
            const profile = await response.json();
            setUserProfile(profile);
          }
        } catch (error) {
          console.error('Error loading user profile:', error);
        }
      } else {
        setUserProfile(null);
      }
      setLoading(false);
    };

    loadProfile();
  }, [session]);

  return (
    <UserProfileContext.Provider value={{
      session,
      userProfile,
      setUserProfile,
      loading,
      isPending
    }}>
      {children}
    </UserProfileContext.Provider>
  );
};

export const useUserProfile = () => {
  const context = useContext(UserProfileContext);
  if (!context) {
    throw new Error('useUserProfile must be used within a UserProfileProvider');
  }
  return context;
};