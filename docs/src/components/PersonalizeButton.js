import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './PersonalizeButton.module.css';

const PersonalizeButton = () => {
  const [loading, setLoading] = useState(false);
  const [personalized, setPersonalized] = useState(false);
  const { user, token } = useAuth();

  const handlePersonalize = async () => {
    const contentElement = document.querySelector('article') || document.querySelector('main');
    if (!contentElement) return;

    if (personalized) {
      window.location.reload();
      return;
    }

    setLoading(true);
    const originalContent = contentElement.innerHTML;
    const userLevel = user?.profile?.software_background || 'beginner';

    try {
      const response = await fetch('http://localhost:8000/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          ...(token && { Authorization: `Bearer ${token}` }),
        },
        body: JSON.stringify({
          content: originalContent,
          user_level: userLevel,
        }),
      });

      if (!response.ok) throw new Error('Personalization failed');

      const data = await response.json();
      contentElement.innerHTML = data.personalized_content;
      setPersonalized(true);
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  // if (!user) return null; // REMOVED: Show button even if logged out

  const handleClick = () => {
    if (!user) {
      alert("Please Sign In to use personalization features!");
      return;
    }
    handlePersonalize();
  };

  return (
    <button
      className={`${styles.personalizeBtn} ${loading ? styles.loading : ''} ${personalized ? styles.active : ''}`}
      onClick={handleClick}
      disabled={loading}
      title={!user ? "Sign in to personalize" : "Personalize content based on your profile"}
    >
      {loading ? 'Personalizing...' :
        personalized ? 'Reset Content' :
          !user ? 'Login to Personalize' :
            'Personalize Content'}
    </button>
  );
};

export default PersonalizeButton;
