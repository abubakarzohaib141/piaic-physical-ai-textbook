import React from 'react';
import ChatBot from '../components/ChatBot';
import AuthNavbar from '../components/AuthNavbar';
import TranslationButton from '../components/TranslationButton';
import TextSelectionPopup from '../components/TextSelectionPopup';
import PersonalizeButton from '../components/PersonalizeButton';
import { AuthProvider } from '../components/AuthContext';

export default function Root({ children }) {
    return (
        <AuthProvider>
            <AuthNavbar />
            {children}
            <PersonalizeButton />
            <TranslationButton />
            <TextSelectionPopup />
            <ChatBot />
        </AuthProvider>
    );
}
