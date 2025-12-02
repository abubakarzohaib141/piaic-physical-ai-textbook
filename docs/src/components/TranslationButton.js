import React, { useState } from 'react';
import styles from './TranslationButton.module.css';

const TranslationButton = () => {
    const [isTranslating, setIsTranslating] = useState(false);
    const [isTranslated, setIsTranslated] = useState(false);

    const handleTranslate = async () => {
        // Find the main content of the documentation page
        const contentElement = document.querySelector('article') || document.querySelector('main');

        if (!contentElement) {
            console.error("No content found to translate");
            return;
        }

        if (isTranslated) {
            // Reload page to revert to English (simplest way to ensure clean state)
            window.location.reload();
            return;
        }

        setIsTranslating(true);
        const originalContent = contentElement.innerHTML;

        try {
            const response = await fetch('http://localhost:8000/translate', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    content: originalContent,
                    target_language: 'ur'
                })
            });

            const data = await response.json();

            if (response.ok) {
                // Replace content with translation
                contentElement.innerHTML = data.translated_content;

                // Add RTL direction for Urdu
                contentElement.style.direction = 'rtl';
                contentElement.style.fontFamily = "'Noto Nastaliq Urdu', 'Arial', sans-serif";

                setIsTranslated(true);
            } else {
                alert('Translation failed: ' + (data.detail || 'Unknown error'));
            }
        } catch (error) {
            console.error('Translation error:', error);
            alert('Failed to connect to translation service');
        } finally {
            setIsTranslating(false);
        }
    };

    return (
        <button
            className={`${styles.translateBtn} ${isTranslating ? styles.loading : ''} ${isTranslated ? styles.active : ''}`}
            onClick={handleTranslate}
            disabled={isTranslating}
        >
            {isTranslating ? (
                <span className={styles.spinner}>↻</span>
            ) : isTranslated ? (
                'Show English'
            ) : (
                'Translate to Urdu'
            )}
        </button>
    );
};

export default TranslationButton;
