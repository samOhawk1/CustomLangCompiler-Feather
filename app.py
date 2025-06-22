from flask import Flask, render_template, request, jsonify
import subprocess
import tempfile
import os

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/rules')
def rules():
    return render_template('rulesection.html')

@app.route('/compile', methods=['POST'])
def compile_code():
    data = request.get_json()
    code = data.get('code')
    action = data.get('action')
    if not code or not action:
        return jsonify({'output': 'Missing code or action.'}), 400

    filename = None
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix='.txt', mode='w') as f:
            f.write(code)
            filename = f.name
        result = subprocess.run(['./main.exe', filename], capture_output=True, text=True, timeout=10)
        output = result.stdout
    except Exception as e:
        output = f"Error running compiler: {e}"
    finally:
        if filename and os.path.exists(filename):
            os.unlink(filename)

    section_titles = {
        "tokenize": "Lexical Analysis (Tokenization)",
        "parse": "Syntax Analysis (Parsing)",
        "semantic": "Semantic Analysis",
        "intermediate": "Intermediate Code Generation",
        "assembly": "Assembly Code Generation",
    }
    section_title = section_titles.get(action)
    if section_title:
        output = extract_section(output, section_title)
    else:
        output = "Invalid action requested."

    return jsonify({'output': output})

def extract_section(text, section_title):
    lines = text.splitlines()
    start = None
    for i, line in enumerate(lines):
        if section_title in line:
            start = i
            break
    if start is None:
        return f"Section '{section_title}' not found."
    end = len(lines)
    for j in range(start + 1, len(lines)):
        if "===" in lines[j] and j != start:
            end = j
            break
    return "\n".join(lines[start:end])

if __name__ == '__main__':
    app.run(debug=True)
