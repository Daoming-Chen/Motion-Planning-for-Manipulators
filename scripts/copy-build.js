const fs = require('fs');
const path = require('path');

function copyDir(src, dest) {
    // Create destination directory if it doesn't exist
    if (!fs.existsSync(dest)) {
        fs.mkdirSync(dest, { recursive: true });
    }
    
    // Read source directory
    const entries = fs.readdirSync(src, { withFileTypes: true });
    
    for (const entry of entries) {
        const srcPath = path.join(src, entry.name);
        const destPath = path.join(dest, entry.name);
        
        if (entry.isDirectory()) {
            copyDir(srcPath, destPath);
        } else {
            fs.copyFileSync(srcPath, destPath);
        }
    }
}

console.log('Copying web demos and models to _build/html...');

// Copy web demos
if (fs.existsSync('dist/web_demos')) {
    copyDir('dist/web_demos', '_build/html/web_demos');
    console.log('✓ Web demos copied successfully');
} else {
    console.warn('⚠ dist/web_demos not found, skipping...');
}

// Copy models
if (fs.existsSync('models')) {
    copyDir('models', '_build/html/models');
    console.log('✓ Models copied successfully');
} else {
    console.warn('⚠ models directory not found, skipping...');
}

console.log('Done!');
