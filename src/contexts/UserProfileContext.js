import React, { createContext, useContext, useEffect, useState } from 'react';

const UserProfileContext = createContext();

export const UserProfileProvider = ({ children }) => {
  const [userProfile, setUserProfile] = useState(null);
  const [loading, setLoading] = useState(true);

  // Load user profile from localStorage when component mounts
  useEffect(() => {
    const loadProfile = () => {
      try {
        const storedProfile = localStorage.getItem('userProfile');
        const isLoggedIn = localStorage.getItem('isLoggedIn');

        if (isLoggedIn === 'true' && storedProfile) {
          const profile = JSON.parse(storedProfile);
          setUserProfile(profile);
        } else {
          setUserProfile(null);
        }
      } catch (error) {
        console.error('Error loading user profile:', error);
        setUserProfile(null);
      } finally {
        setLoading(false);
      }
    };

    loadProfile();

    // Set up storage event listener for cross-tab synchronization
    const handleStorageChange = () => {
      loadProfile();
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, []);

  const refreshProfile = () => {
    const storedProfile = localStorage.getItem('userProfile');
    if (storedProfile) {
      const profile = JSON.parse(storedProfile);
      setUserProfile(profile);
    }
  };

  const signOut = () => {
    localStorage.removeItem('userProfile');
    localStorage.removeItem('isLoggedIn');
    localStorage.removeItem('authToken');
    setUserProfile(null);
  };

  return (
    <UserProfileContext.Provider value={{
      userProfile,
      setUserProfile,
      loading,
      refreshProfile,
      signOut,
      session: userProfile ? { user: { id: userProfile.user_id, name: userProfile.name, email: userProfile.email } } : null,
      isPending: false
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