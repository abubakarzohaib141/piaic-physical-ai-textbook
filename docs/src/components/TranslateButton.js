import React, { useState } from 'react';
import styles from './ChapterButtons.module.css';

export default function TranslateButton({ content, onTranslated }) {
  const [loading, setLoading] = useState(false);
  const [isUrdu, setIsUrdu] = useState(false);
  const [originalContent] = useState(content);

  const API_URL = 'http://localhost:8000';

  const handleToggleTranslation = async () => {
    if (isUrdu) {
      // Switch back to English
      setIsUrdu(false);
      if (onTranslated) {
        onTranslated(originalContent, false);
      }
      return;
    }

    // Translate to Urdu
    if (!content) return;

    setLoading(true);

    try {
      const response = await fetch(`${API_URL}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content,
          target_language: 'ur',
        }),
      });

      if (!response.ok) throw new Error('Translation failed');

      const data = await response.json();

      setIsUrdu(true);
      if (onTranslated) {
        onTranslated(data.translated_content, true, data.cached);
      }
    } catch (error) {
      console.error('Error translating content:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <button
      className={`${styles.chapterButton} ${styles.translateButton}`}
      onClick={handleToggleTranslation}
      disabled={loading}
    >
      {loading ? 'Translating...' : isUrdu ? 'Show English' : 'Translate to Urdu'}
    </button>
  );
}
