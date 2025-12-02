from flask import Flask, render_template_string, jsonify
from vector_store import vector_store
import sys

app = Flask(__name__)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Qdrant Database Viewer</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
            margin: 0;
            padding: 20px;
            background: #f5f5f5;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        h1 {
            color: #10a37f;
            margin-bottom: 10px;
        }
        .stats {
            background: #f0f0f0;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
        }
        .document {
            border: 1px solid #e0e0e0;
            padding: 15px;
            margin-bottom: 15px;
            border-radius: 8px;
            background: #fafafa;
        }
        .doc-id {
            color: #666;
            font-size: 12px;
            margin-bottom: 8px;
        }
        .doc-source {
            color: #10a37f;
            font-weight: 600;
            margin-bottom: 8px;
        }
        .doc-text {
            color: #333;
            line-height: 1.6;
        }
        .error {
            color: #d32f2f;
            padding: 20px;
            background: #ffebee;
            border-radius: 8px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>📊 Qdrant Vector Database</h1>
        <div id="content">Loading...</div>
    </div>
    <script>
        fetch('/api/documents')
            .then(r => r.json())
            .then(data => {
                if (data.error) {
                    document.getElementById('content').innerHTML = 
                        `<div class="error">${data.error}</div>`;
                    return;
                }
                
                let html = `
                    <div class="stats">
                        <strong>Collection:</strong> ${data.collection}<br>
                        <strong>Total Documents:</strong> ${data.total}<br>
                        <strong>Vector Size:</strong> ${data.vector_size} dimensions
                    </div>
                    <h2>Documents (showing first 50):</h2>
                `;
                
                data.documents.forEach((doc, i) => {
                    html += `
                        <div class="document">
                            <div class="doc-id">ID: ${doc.id}</div>
                            <div class="doc-source">📄 ${doc.source}</div>
                            <div class="doc-text">${doc.text}</div>
                        </div>
                    `;
                });
                
                document.getElementById('content').innerHTML = html;
            })
            .catch(err => {
                document.getElementById('content').innerHTML = 
                    `<div class="error">Error: ${err.message}</div>`;
            });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/documents')
def get_documents():
    try:
        collection = vector_store.client.get_collection(vector_store.collection_name)
        
        results, _ = vector_store.client.scroll(
            collection_name=vector_store.collection_name,
            limit=50,
            with_payload=True,
            with_vectors=False
        )
        
        documents = []
        for point in results:
            documents.append({
                'id': point.id,
                'source': point.payload.get('metadata', {}).get('file_path', 'Unknown'),
                'text': point.payload.get('text', '')[:300] + '...'
            })
        
        return jsonify({
            'collection': collection.name,
            'total': collection.points_count,
            'vector_size': collection.config.params.vectors.size,
            'documents': documents
        })
    except Exception as e:
        return jsonify({'error': str(e)})

if __name__ == '__main__':
    print("\n" + "="*60)
    print("🌐 Qdrant Database Viewer")
    print("="*60)
    print("\nOpen in browser: http://localhost:5000")
    print("\nPress Ctrl+C to stop\n")
    app.run(debug=True, port=5000)
