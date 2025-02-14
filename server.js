const express = require('express');
const { Pool } = require('pg');
const cors = require('cors');
const path = require('path');

const app = express();
app.use(cors());

// PostgreSQL bağlantı bilgileri
const pool = new Pool({
    user: 'postgres',
    host: 'localhost',
    database: 'volcano_db',
    password: 'hdrklc123', // PostgreSQL kurulumunda belirlediğiniz şifre
    port: 5432,
});

// Bağlantı testi ekleyelim
pool.query('SELECT NOW()', (err, res) => {
    if (err) {
        console.error('PostgreSQL bağlantı hatası:', err);
    } else {
        console.log('PostgreSQL bağlantısı başarılı');
    }
});

// Statik dosyaları servis et
app.use(express.static(path.join(__dirname, 'public')));

// Ana sayfa için route - index.html artık public klasöründe
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Tüm volkanları getiren endpoint
app.get('/api/volcanoes', async (req, res) => {
    try {
        const result = await pool.query('SELECT * FROM volcanoes');
        
        // Sütun isimlerini orijinal halleriyle kullan
        console.log('Sample row:', result.rows[0]);
        res.json(result.rows);
    } catch (err) {
        console.error('Database query error:', err);
        res.status(500).json({ error: 'Database error', details: err.message });
    }
});

// Sunucuyu başlat
const PORT = 3000;
app.listen(PORT, () => {
    console.log(`Sunucu ${PORT} portunda çalışıyor`);
});
