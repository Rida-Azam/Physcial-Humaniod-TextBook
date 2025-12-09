import asyncpg
import os
from typing import Dict, Any, Optional
import json

class NeonService:
    def __init__(self):
        self.connection_string = os.getenv("NEON_CONNECTION_STRING")
        self.pool = None

    async def initialize(self):
        """
        Initialize connection pool
        """
        if self.connection_string:
            self.pool = await asyncpg.create_pool(self.connection_string)

    async def get_user_preferences(self, user_id: str) -> Dict[str, Any]:
        """
        Get user preferences from Neon PostgreSQL
        """
        if not self.pool:
            return {}

        async with self.pool.acquire() as connection:
            result = await connection.fetchrow(
                "SELECT preferences FROM users WHERE user_id = $1",
                user_id
            )
            if result:
                return result['preferences'] or {}
            else:
                # Create new user with default preferences
                await connection.execute(
                    "INSERT INTO users (user_id, preferences) VALUES ($1, $2)",
                    user_id, json.dumps({})
                )
                return {}

    async def set_user_preferences(self, user_id: str, preferences: Dict[str, Any]) -> bool:
        """
        Set user preferences in Neon PostgreSQL
        """
        if not self.pool:
            return False

        try:
            async with self.pool.acquire() as connection:
                await connection.execute(
                    """
                    INSERT INTO users (user_id, preferences) VALUES ($1, $2)
                    ON CONFLICT (user_id)
                    DO UPDATE SET preferences = $2, updated_at = NOW()
                    """,
                    user_id, json.dumps(preferences)
                )
                return True
        except Exception:
            return False

    async def get_user_progress(self, user_id: str) -> Dict[str, Any]:
        """
        Get user progress from Neon PostgreSQL
        """
        if not self.pool:
            return {}

        async with self.pool.acquire() as connection:
            result = await connection.fetch(
                "SELECT chapter_id, progress, completed FROM user_progress WHERE user_id = $1",
                user_id
            )
            progress = {}
            for row in result:
                progress[row['chapter_id']] = {
                    'progress': row['progress'],
                    'completed': row['completed']
                }
            return progress

    async def set_user_progress(self, user_id: str, chapter_id: str, progress: float, completed: bool) -> bool:
        """
        Set user progress in Neon PostgreSQL
        """
        if not self.pool:
            return False

        try:
            async with self.pool.acquire() as connection:
                await connection.execute(
                    """
                    INSERT INTO user_progress (user_id, chapter_id, progress, completed)
                    VALUES ($1, $2, $3, $4)
                    ON CONFLICT (user_id, chapter_id)
                    DO UPDATE SET progress = $3, completed = $4, updated_at = NOW()
                    """,
                    user_id, chapter_id, progress, completed
                )
                return True
        except Exception:
            return False

    async def get_translation_data(self, lang_from: str, lang_to: str, text_hash: str) -> Optional[str]:
        """
        Get cached translation from Neon PostgreSQL
        """
        if not self.pool:
            return None

        async with self.pool.acquire() as connection:
            result = await connection.fetchrow(
                """
                SELECT translated_text FROM translations
                WHERE lang_from = $1 AND lang_to = $2 AND text_hash = $3
                """,
                lang_from, lang_to, text_hash
            )
            return result['translated_text'] if result else None

    async def cache_translation(self, lang_from: str, lang_to: str, text_hash: str, translated_text: str) -> bool:
        """
        Cache translation in Neon PostgreSQL
        """
        if not self.pool:
            return False

        try:
            async with self.pool.acquire() as connection:
                await connection.execute(
                    """
                    INSERT INTO translations (lang_from, lang_to, text_hash, translated_text)
                    VALUES ($1, $2, $3, $4)
                    ON CONFLICT (lang_from, lang_to, text_hash)
                    DO UPDATE SET translated_text = $4, updated_at = NOW()
                    """,
                    lang_from, lang_to, text_hash, translated_text
                )
                return True
        except Exception:
            return False