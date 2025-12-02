import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';
import styles from './TextSelectionPopup.module.css';

const TextSelectionPopup = () => {
    const [position, setPosition] = useState(null);
    const [selectedText, setSelectedText] = useState('');
    const [show, setShow] = useState(false);

    useEffect(() => {
        const handleSelectionChange = () => {
            const selection = window.getSelection();
            const text = selection.toString().trim();

            if (text.length > 0) {
                const range = selection.getRangeAt(0);
                const rect = range.getBoundingClientRect();

                // Calculate position: centered above the selection
                setPosition({
                    top: rect.top + window.scrollY - 40, // 40px above
                    left: rect.left + window.scrollX + (rect.width / 2)
                });
                setSelectedText(text);
                setShow(true);
            } else {
                setShow(false);
            }
        };

        // Use mouseup to detect end of selection
        document.addEventListener('mouseup', handleSelectionChange);
        // Also listen to selectionchange for more responsiveness, but debounce might be needed
        // For now, mouseup is safer to avoid flickering while dragging

        // Hide on scroll to avoid misalignment
        document.addEventListener('scroll', () => setShow(false));

        return () => {
            document.removeEventListener('mouseup', handleSelectionChange);
            document.removeEventListener('scroll', () => setShow(false));
        };
    }, []);

    const handleAsk = (e) => {
        e.stopPropagation(); // Prevent clearing selection immediately
        const event = new CustomEvent('open-chat', {
            detail: { selectedText }
        });
        window.dispatchEvent(event);
        setShow(false);
        window.getSelection().removeAllRanges(); // Clear selection after asking
    };

    if (!show || !position) return null;

    return ReactDOM.createPortal(
        <button
            className={styles.askButton}
            style={{ top: position.top, left: position.left }}
            onClick={handleAsk}
            onMouseDown={(e) => e.preventDefault()} // Prevent button click from clearing selection
        >
            <span className={styles.icon}>💬</span>
            Ask AI
        </button>,
        document.body
    );
};

export default TextSelectionPopup;
