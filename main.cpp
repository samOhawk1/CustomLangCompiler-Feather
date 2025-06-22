#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cctype>
#include <cstring>
#include <utility> // For std::move

// Token types
enum class TokenType {
    VAR, PRINT, IF, ELSE, RETURN,
    ID, NUMBER, STRING,
    OP, COMPARE, ASSIGN,
    LPAREN, RPAREN, LBRACE, RBRACE, SEMI,
    END
};

// Token structure
struct Token {
    TokenType type;
    std::string value;
    int line;
    int column;

    Token(TokenType t, const std::string& v, int l, int c)
        : type(t), value(v), line(l), column(c) {}
};

// AST Node structure
struct ASTNode {
    std::string nodeType;
    std::vector<std::unique_ptr<ASTNode>> children;
    std::string value;
    std::string dataType; // Added for semantic analysis (e.g., "int", "string")
    int indentLevel = 0;

    ASTNode(std::string type, std::string val = "", std::string dType = "")
        : nodeType(std::move(type)), value(std::move(val)), dataType(std::move(dType)) {}

    void addChild(std::unique_ptr<ASTNode> child) {
        child->indentLevel = indentLevel + 1;
        children.push_back(std::move(child));
    }
};

// Compiler class
class MiniCompiler {
private:
    std::vector<Token> tokens;
    size_t currentTokenIndex = 0;
    // Changed symbol table to store type as well
    std::map<std::string, std::string> symbolTable; // varName -> type (e.g., "int", "string")
    std::vector<std::string> errors;
    std::vector<std::string> intermediateCode;
    std::vector<std::string> assemblyCode;
    std::unique_ptr<ASTNode> ast;

    // Helper functions
    void addError(const std::string& error) { errors.push_back(error); }
    void addToken(TokenType type, const std::string& value, int line, int column) { tokens.emplace_back(type, value, line, column); }
    void addIntermediateCode(const std::string& code) { intermediateCode.push_back(code); }
    void addAssemblyCode(const std::string& code) { assemblyCode.push_back(code); }
    // Modified to store type
    void addSymbol(const std::string& key, const std::string& type) { symbolTable[key] = type; }
    bool symbolExists(const std::string& key) const { return symbolTable.count(key) > 0; }
    std::string getSymbolType(const std::string& key) const {
        auto it = symbolTable.find(key);
        if (it != symbolTable.end()) {
            return it->second;
        }
        return ""; // Not found
    }


    // Tokenizer
    void tokenize(const std::string& source);

    // Parser
    const Token& currentToken() const;
    void advance();
    int getPrecedence(const std::string& op) const;
    std::unique_ptr<ASTNode> parseExpression(int minPrec);
    std::unique_ptr<ASTNode> parsePrimary();
    std::unique_ptr<ASTNode> parseDeclaration();
    std::unique_ptr<ASTNode> parsePrint();
    std::unique_ptr<ASTNode> parseReturn();
    std::unique_ptr<ASTNode> parseIfElse();
    std::unique_ptr<ASTNode> parseProgramBlock();
    std::unique_ptr<ASTNode> parseProgram();

    // Semantic Analysis
    std::string semanticAnalyzeExpression(ASTNode* node); // Returns type of expression
    void semanticAnalysis(ASTNode* node);

    // Intermediate Code Generation
    std::string generateIntermediateCode(ASTNode* node, int& tempCount, int& labelCount); // Returns the result variable/value

    // Assembly Code Generation
    void generateAssembly(ASTNode* node, int& regCount);

    // Print functions
    void printTokens() const;
    void printErrors() const;
    void printAST(const ASTNode* node) const;
    void printASTJSON(const ASTNode* node, std::ostream& out, int indent) const;

public:
    void compile(const std::string& filename);
    bool hasErrors() const { return !errors.empty(); }
};

// Implementation

void MiniCompiler::compile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error: Could not open file '" << filename << "'" << std::endl;
        return;
    }

    std::string sourceCode((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
    file.close();

    std::cout << "=== Source Code ===" << std::endl;
    std::cout << sourceCode << std::endl << std::endl;

    tokens.clear();
    currentTokenIndex = 0;
    symbolTable.clear();
    errors.clear();
    intermediateCode.clear();
    assemblyCode.clear();
    ast.reset();

    std::cout << "=== Lexical Analysis (Tokenization) ===" << std::endl;
    tokenize(sourceCode);
    printTokens();

    if (!errors.empty()) {
        printErrors();
        return;
    }

    std::cout << "\n=== Syntax Analysis (Parsing) ===" << std::endl;
    ast = parseProgram();

    if (!errors.empty()) {
        printErrors();
        return;
    }

    std::cout << "\nParse Tree:" << std::endl;
    if (ast) printAST(ast.get());

    std::cout << "\nParse Tree (JSON):" << std::endl;
    if (ast) printASTJSON(ast.get(), std::cout, 0);
    std::cout << std::endl;

    std::cout << "\n=== Semantic Analysis ===" << std::endl;
    semanticAnalysis(ast.get());

    if (!errors.empty()) {
        printErrors();
        return;
    }

    std::cout << "\nSymbol Table:" << std::endl;
    for (const auto& entry : symbolTable) {
        std::cout << entry.first << ": " << entry.second << std::endl;
    }

    std::cout << "\n=== Intermediate Code Generation ===" << std::endl;
    int tempCount = 0;
    int labelCount = 0;
    generateIntermediateCode(ast.get(), tempCount, labelCount);

    std::cout << "\nIntermediate Code (Three-Address Code):" << std::endl;
    for (size_t i = 0; i < intermediateCode.size(); i++) {
        std::cout << i << ": " << intermediateCode[i] << std::endl;
    }

    std::cout << "\n=== Assembly Code Generation ===" << std::endl;
    int regCount = 0;
    // Add data section for strings and variables
    addAssemblyCode("section .data");
    for(const auto& entry : symbolTable) {
        if (entry.second == "int") {
            addAssemblyCode(entry.first + " dd 0"); // Define a double word for int variables
        } else if (entry.second == "string") {
            // For simplicity, we'll store string literals inline in .data
            // Real compilers would manage a string pool.
            // Placeholder: Assume strings passed to print are immediate.
        }
    }
    // Add any specific string literals as data
    // This is a basic approach; a real compiler would optimize string constant storage.
    // For now, if string literals are processed directly, they might not need explicit .data entries
    // unless they are assigned to variables.
    // Example: global_string_0 db "hello world", 0
    // We will generate these as needed during assembly generation if they are used by print directly.

    addAssemblyCode("section .text");
    addAssemblyCode("global _start"); // For Linux executables

    addAssemblyCode("_start:"); // Entry point
    generateAssembly(ast.get(), regCount);
    addAssemblyCode("mov eax, 1"); // sys_exit
    addAssemblyCode("xor ebx, ebx"); // exit code 0
    addAssemblyCode("int 0x80"); // call kernel

    // Basic print_int and print_string implementations (for Linux x86 NASM)
    // These are highly simplified and would typically come from a runtime library.
    addAssemblyCode("");
    addAssemblyCode("print_int:");
    addAssemblyCode("    push ebp");
    addAssemblyCode("    mov ebp, esp");
    addAssemblyCode("    push eax"); // Value to print
    addAssemblyCode("    mov ecx, esp");
    addAssemblyCode("    sub esp, 4"); // For character buffer
    addAssemblyCode("    mov edi, esp");
    addAssemblyCode("    mov esi, 0"); // Length counter
    addAssemblyCode("    cmp dword [ecx], 0");
    addAssemblyCode("    jge .print_pos");
    addAssemblyCode("    neg dword [ecx]");
    addAssemblyCode("    mov byte [edi], '-'");
    addAssemblyCode("    inc edi");
    addAssemblyCode("    inc esi");
    addAssemblyCode(".print_pos:");
    addAssemblyCode("    mov ebx, 10");
    addAssemblyCode("    mov edx, 0");
    addAssemblyCode("    div ebx");
    addAssemblyCode("    add edx, '0'");
    addAssemblyCode("    push edx");
    addAssemblyCode("    inc esi");
    addAssemblyCode("    cmp eax, 0");
    addAssemblyCode("    jne .print_pos");
    addAssemblyCode(".print_loop:");
    addAssemblyCode("    pop edx");
    addAssemblyCode("    mov byte [edi], dl");
    addAssemblyCode("    inc edi");
    addAssemblyCode("    loop .print_loop");
    addAssemblyCode("    mov eax, 4"); // sys_write
    addAssemblyCode("    mov ebx, 1"); // stdout
    addAssemblyCode("    mov ecx, esp");
    addAssemblyCode("    mov edx, esi");
    addAssemblyCode("    int 0x80");
    addAssemblyCode("    add esp, 4");
    addAssemblyCode("    pop eax"); // Restore original eax
    addAssemblyCode("    pop ebp");
    addAssemblyCode("    ret");

    addAssemblyCode("");
    addAssemblyCode("print_string:");
    addAssemblyCode("    push ebp");
    addAssemblyCode("    mov ebp, esp");
    addAssemblyCode("    push eax"); // String address
    addAssemblyCode("    mov ecx, eax");
    addAssemblyCode("    xor edx, edx"); // Length
    addAssemblyCode(".string_len_loop:");
    addAssemblyCode("    cmp byte [ecx + edx], 0");
    addAssemblyCode("    je .string_len_end");
    addAssemblyCode("    inc edx");
    addAssemblyCode("    jmp .string_len_loop");
    addAssemblyCode(".string_len_end:");
    addAssemblyCode("    mov eax, 4"); // sys_write
    addAssemblyCode("    mov ebx, 1"); // stdout
    addAssemblyCode("    int 0x80");
    addAssemblyCode("    pop eax"); // Restore original eax
    addAssemblyCode("    pop ebp");
    addAssemblyCode("    ret");


    for (const auto& code : assemblyCode) {
        std::cout << code << std::endl;
    }

    std::cout << "\nCompilation successful!" << std::endl;
}

void MiniCompiler::tokenize(const std::string& source) {
    size_t pos = 0;
    int line = 1;
    int lineStart = 0;
    size_t sourceLen = source.length();

    while (pos < sourceLen) {
        while (pos < sourceLen && std::isspace(source[pos])) {
            if (source[pos] == '\n') {
                line++;
                lineStart = pos + 1;
            }
            pos++;
        }
        if (pos >= sourceLen) break;

        int tokenStart = pos;
        TokenType type = TokenType::END;
        std::string value;

        if (std::isalpha(source[pos]) || source[pos] == '_') {
            while (pos < sourceLen && (std::isalnum(source[pos]) || source[pos] == '_')) {
                pos++;
            }
            value = source.substr(tokenStart, pos - tokenStart);

            if (value == "var") type = TokenType::VAR;
            else if (value == "print") type = TokenType::PRINT;
            else if (value == "if") type = TokenType::IF;
            else if (value == "else") type = TokenType::ELSE;
            else if (value == "return") type = TokenType::RETURN;
            else type = TokenType::ID;
        }
        else if (std::isdigit(source[pos])) {
            while (pos < sourceLen && std::isdigit(source[pos])) {
                pos++;
            }
            value = source.substr(tokenStart, pos - tokenStart);
            type = TokenType::NUMBER;
        }
        else if (source[pos] == '"') {
            pos++;
            tokenStart = pos;
            while (pos < sourceLen && source[pos] != '"') {
                pos++;
            }
            if (pos >= sourceLen) {
                addError("Unterminated string literal at line " + std::to_string(line) + ", column " + std::to_string(tokenStart - lineStart));
                break;
            }
            value = source.substr(tokenStart, pos - tokenStart);
            type = TokenType::STRING;
            pos++;
        }
        else if (std::strchr("+-*/", source[pos])) {
            value = source.substr(pos, 1);
            pos++;
            type = TokenType::OP;
        }
        else if (std::strchr("=!<>", source[pos])) {
            if (pos + 1 < sourceLen && source[pos+1] == '=') {
                value = source.substr(pos, 2);
                pos += 2;
                type = TokenType::COMPARE;
            } else if (source[pos] == '=') { // Handle single '=' for assignment separately from comparisons
                value = source.substr(pos, 1);
                pos++;
                type = TokenType::ASSIGN;
            }
            else { // Single < or >
                value = source.substr(pos, 1);
                pos++;
                type = TokenType::COMPARE;
            }
        }
        else {
            switch (source[pos]) {
                case '(': type = TokenType::LPAREN; value = source.substr(pos, 1); pos++; break;
                case ')': type = TokenType::RPAREN; value = source.substr(pos, 1); pos++; break;
                case '{': type = TokenType::LBRACE; value = source.substr(pos, 1); pos++; break;
                case '}': type = TokenType::RBRACE; value = source.substr(pos, 1); pos++; break;
                case ';': type = TokenType::SEMI; value = source.substr(pos, 1); pos++; break;
                default: {
                    std::string error = "Illegal character '\\x";
                    char buf[3];
                    snprintf(buf, sizeof(buf), "%02x", (unsigned char)source[pos]);
                    error += buf;
                    error += "' at line " + std::to_string(line) +
                                ", column " + std::to_string(pos - lineStart);
                    addError(error);
                    pos++;
                    continue;
                }
            }
        }

        addToken(type, value, line, tokenStart - lineStart);
    }
    addToken(TokenType::END, "", line, 0);
}

const Token& MiniCompiler::currentToken() const {
    static const Token endToken{TokenType::END, "", 0, 0};
    return (currentTokenIndex < tokens.size()) ? tokens[currentTokenIndex] : endToken;
}

void MiniCompiler::advance() {
    if (currentTokenIndex < tokens.size()) { // Allow advancing past the last token for END
        currentTokenIndex++;
    }
}

int MiniCompiler::getPrecedence(const std::string& op) const {
    if (op.empty()) return -1;
    if (op == "+" || op == "-") return 1;
    if (op == "*" || op == "/") return 2;
    if (op == "<" || op == ">" || op == "<=" || op == ">=" || op == "==" || op == "!=") return 0;
    return -1;
}

std::unique_ptr<ASTNode> MiniCompiler::parsePrimary() {
    const Token& token = currentToken();
    std::unique_ptr<ASTNode> node;

    if (token.type == TokenType::ID) {
        node = std::make_unique<ASTNode>("Identifier", token.value);
        advance();
    }
    else if (token.type == TokenType::NUMBER) {
        node = std::make_unique<ASTNode>("NumberLiteral", token.value, "int"); // Assign type "int"
        advance();
    }
    else if (token.type == TokenType::STRING) { // Added for string literals
        node = std::make_unique<ASTNode>("StringLiteral", token.value, "string"); // Assign type "string"
        advance();
    }
    else if (token.type == TokenType::LPAREN) {
        advance();
        node = parseExpression(0);
        if (!node) return nullptr;
        if (currentToken().type != TokenType::RPAREN) {
            addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ')' after expression");
            return nullptr;
        }
        advance();
    }
    else {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected identifier, number, string, or '('");
        return nullptr;
    }
    return node;
}

std::unique_ptr<ASTNode> MiniCompiler::parseExpression(int minPrec) {
    auto left = parsePrimary();
    if (!left) return nullptr;

    while (true) {
        const Token& token = currentToken();
        if (token.type != TokenType::OP && token.type != TokenType::COMPARE) break;

        std::string op = token.value;
        int prec = getPrecedence(op);
        if (prec < minPrec) break;

        // Special handling for assignment: it's right-associative and low precedence
        if (token.type == TokenType::ASSIGN) {
            // Assignment is handled as a statement, not an expression in this grammar,
            // so this path won't be taken for 'ID = Expr' directly.
            // If we allow 'a = b = 5', this needs more thought.
            // For now, expressions are arithmetic/comparison.
            break;
        }

        advance();
        auto right = parseExpression(prec + 1); // For binary operators, recursive call to parse right operand
        if (!right) return nullptr;

        auto bin = std::make_unique<ASTNode>("BinaryExpr", op);
        bin->addChild(std::move(left));
        bin->addChild(std::move(right));
        left = std::move(bin);
    }
    return left;
}

std::unique_ptr<ASTNode> MiniCompiler::parseDeclaration() {
    advance(); // skip 'var'
    if (currentToken().type != TokenType::ID) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected identifier after 'var'");
        return nullptr;
    }

    std::string varName = currentToken().value;
    advance();
    std::unique_ptr<ASTNode> expr;

    if (currentToken().type == TokenType::ASSIGN) {
        advance();
        expr = parseExpression(0); // Parse the expression for assignment
        if (!expr) {
            addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Invalid expression in declaration assignment");
            return nullptr;
        }
    }

    if (currentToken().type != TokenType::SEMI) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ';' at end of declaration");
        return nullptr;
    }
    advance();

    auto decl = std::make_unique<ASTNode>("Declaration", varName);
    if (expr) decl->addChild(std::move(expr)); // The expression is the initializer
    return decl;
}

std::unique_ptr<ASTNode> MiniCompiler::parsePrint() {
    advance(); // skip 'print'
    if (currentToken().type != TokenType::LPAREN) { // Assume print takes an expression in parentheses
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected '(' after 'print'");
        return nullptr;
    }
    advance();

    auto expr = parseExpression(0);
    if (!expr) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Invalid expression in print statement");
        return nullptr;
    }

    if (currentToken().type != TokenType::RPAREN) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ')' after print expression");
        return nullptr;
    }
    advance();


    if (currentToken().type != TokenType::SEMI) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ';' after print statement");
        return nullptr;
    }
    advance();

    auto node = std::make_unique<ASTNode>("Print");
    node->addChild(std::move(expr));
    return node;
}

std::unique_ptr<ASTNode> MiniCompiler::parseReturn() {
    advance(); // skip 'return'
    auto expr = parseExpression(0);
    if (!expr) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Invalid expression in return statement");
        return nullptr;
    }
    if (currentToken().type != TokenType::SEMI) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ';' after 'return'");
        return nullptr;
    }
    advance();

    auto node = std::make_unique<ASTNode>("Return");
    node->addChild(std::move(expr));
    return node;
}

std::unique_ptr<ASTNode> MiniCompiler::parseIfElse() {
    advance(); // skip 'if'
    if (currentToken().type != TokenType::LPAREN) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected '(' after 'if'");
        return nullptr;
    }
    advance();

    auto cond = parseExpression(0);
    if (!cond) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Invalid condition in if statement");
        return nullptr;
    }
    if (currentToken().type != TokenType::RPAREN) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ')' after condition");
        return nullptr;
    }
    advance();

    if (currentToken().type != TokenType::LBRACE) {
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected '{' after 'if' condition");
        return nullptr;
    }
    advance();

    auto ifBlock = parseProgramBlock();
    if (!ifBlock) return nullptr;

    if (currentToken().type == TokenType::ELSE) {
        advance();
        if (currentToken().type != TokenType::LBRACE) {
            addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected '{' after 'else'");
            return nullptr;
        }
        advance();

        auto elseBlock = parseProgramBlock();
        if (!elseBlock) return nullptr;

        auto ifNode = std::make_unique<ASTNode>("IfElse");
        ifNode->addChild(std::move(cond));
        ifNode->addChild(std::move(ifBlock));
        ifNode->addChild(std::move(elseBlock));
        return ifNode;
    } else {
        auto ifNode = std::make_unique<ASTNode>("If");
        ifNode->addChild(std::move(cond));
        ifNode->addChild(std::move(ifBlock));
        return ifNode;
    }
}

std::unique_ptr<ASTNode> MiniCompiler::parseProgramBlock() {
    auto block = std::make_unique<ASTNode>("Block");
    while (currentToken().type != TokenType::RBRACE && currentToken().type != TokenType::END) {
        switch (currentToken().type) {
            case TokenType::VAR:
                if (auto decl = parseDeclaration()) block->addChild(std::move(decl));
                break;
            case TokenType::PRINT:
                if (auto prt = parsePrint()) block->addChild(std::move(prt));
                break;
            case TokenType::IF:
                if (auto ifstmt = parseIfElse()) block->addChild(std::move(ifstmt));
                break;
            case TokenType::RETURN:
                if (auto ret = parseReturn()) block->addChild(std::move(ret));
                break;
            case TokenType::ID: { // Handle assignment statements like 'a = 5;'
                std::string varName = currentToken().value;
                advance();
                if (currentToken().type != TokenType::ASSIGN) {
                    addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected '=' for assignment");
                    return nullptr; // Error, break parsing
                }
                advance(); // Skip '='
                auto expr = parseExpression(0);
                if (!expr) {
                    addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Invalid expression in assignment");
                    return nullptr; // Error, break parsing
                }
                if (currentToken().type != TokenType::SEMI) {
                    addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ';' at end of assignment");
                    return nullptr; // Error, break parsing
                }
                advance(); // Skip ';'
                auto assignNode = std::make_unique<ASTNode>("Assignment", varName);
                assignNode->addChild(std::move(expr));
                block->addChild(std::move(assignNode));
                break;
            }
            default:
                addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Unexpected token in block: " + currentToken().value);
                advance(); // Try to recover by advancing past the unexpected token
                break;
        }
    }
    if (currentToken().type == TokenType::RBRACE) advance();
    else if (currentToken().type != TokenType::END) { // If it's not RBRACE and not END, something is wrong
        addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected '}' at end of block");
    }
    return block;
}

std::unique_ptr<ASTNode> MiniCompiler::parseProgram() {
    auto program = std::make_unique<ASTNode>("Program");
    while (currentToken().type != TokenType::END) {
        switch (currentToken().type) {
            case TokenType::VAR:
                if (auto decl = parseDeclaration()) program->addChild(std::move(decl));
                else {
                    // If parsing failed, advance to try to recover and find next statement
                    // This is a simplistic error recovery.
                    while (currentToken().type != TokenType::SEMI && currentToken().type != TokenType::END) {
                        advance();
                    }
                    if (currentToken().type == TokenType::SEMI) advance();
                }
                break;
            case TokenType::PRINT:
                if (auto prt = parsePrint()) program->addChild(std::move(prt));
                else {
                     while (currentToken().type != TokenType::SEMI && currentToken().type != TokenType::END) {
                        advance();
                    }
                    if (currentToken().type == TokenType::SEMI) advance();
                }
                break;
            case TokenType::IF:
                if (auto ifstmt = parseIfElse()) program->addChild(std::move(ifstmt));
                else {
                     while (currentToken().type != TokenType::RBRACE && currentToken().type != TokenType::SEMI && currentToken().type != TokenType::END) {
                        advance();
                    }
                    if (currentToken().type == TokenType::RBRACE) advance();
                    if (currentToken().type == TokenType::SEMI) advance();
                }
                break;
            case TokenType::RETURN:
                if (auto ret = parseReturn()) program->addChild(std::move(ret));
                else {
                     while (currentToken().type != TokenType::SEMI && currentToken().type != TokenType::END) {
                        advance();
                    }
                    if (currentToken().type == TokenType::SEMI) advance();
                }
                break;
            case TokenType::ID: { // Handle assignment statements at program level
                std::string varName = currentToken().value;
                advance();
                if (currentToken().type != TokenType::ASSIGN) {
                    addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected '=' for assignment");
                    // Simple error recovery: advance until next semicolon or end
                    while (currentToken().type != TokenType::SEMI && currentToken().type != TokenType::END) {
                        advance();
                    }
                    if (currentToken().type == TokenType::SEMI) advance();
                    break;
                }
                advance(); // Skip '='
                auto expr = parseExpression(0);
                if (!expr) {
                    addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Invalid expression in assignment");
                    while (currentToken().type != TokenType::SEMI && currentToken().type != TokenType::END) {
                        advance();
                    }
                    if (currentToken().type == TokenType::SEMI) advance();
                    break;
                }
                if (currentToken().type != TokenType::SEMI) {
                    addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Expected ';' at end of assignment");
                     while (currentToken().type != TokenType::SEMI && currentToken().type != TokenType::END) {
                        advance();
                    }
                    if (currentToken().type == TokenType::SEMI) advance();
                    break;
                }
                advance(); // Skip ';'
                auto assignNode = std::make_unique<ASTNode>("Assignment", varName);
                assignNode->addChild(std::move(expr));
                program->addChild(std::move(assignNode));
                break;
            }
            default:
                addError("Line " + std::to_string(currentToken().line) + ", Column " + std::to_string(currentToken().column) + ": Unexpected token in program: " + currentToken().value);
                advance(); // Try to recover by advancing past the unexpected token
                break;
        }
    }
    return program;
}


std::string MiniCompiler::semanticAnalyzeExpression(ASTNode* node) {
    if (!node) return "error";

    if (node->nodeType == "NumberLiteral") {
        node->dataType = "int";
        return "int";
    }
    else if (node->nodeType == "StringLiteral") {
        node->dataType = "string";
        return "string";
    }
    else if (node->nodeType == "Identifier") {
        if (!symbolExists(node->value)) {
            addError("Undeclared variable '" + node->value + "' at line (unknown), column (unknown)."); // Line/Col info is lost here from token
            return "error";
        }
        node->dataType = getSymbolType(node->value);
        return node->dataType;
    }
    else if (node->nodeType == "BinaryExpr") {
        std::string leftType = semanticAnalyzeExpression(node->children[0].get());
        std::string rightType = semanticAnalyzeExpression(node->children[1].get());

        if (leftType == "error" || rightType == "error") {
            return "error";
        }

        if (leftType != rightType) {
            addError("Type mismatch in binary expression '" + node->value + "': " + leftType + " vs " + rightType);
            return "error";
        }

        // For arithmetic ops, result is int. For comparison ops, result is bool (represented as int 0/1)
        if (node->value == "+" || node->value == "-" || node->value == "*" || node->value == "/") {
            if (leftType != "int") {
                 addError("Arithmetic operations only supported for integers. Found " + leftType);
                 return "error";
            }
            node->dataType = "int";
            return "int";
        } else if (node->value == "==" || node->value == "!=" || node->value == "<" || node->value == ">" || node->value == "<=" || node->value == ">=") {
            // Comparisons can work on strings too if defined, but for now assume int comparison
            if (leftType != "int") {
                 addError("Comparison operations only supported for integers. Found " + leftType);
                 return "error";
            }
            node->dataType = "int"; // Boolean result (0 or 1)
            return "int";
        }
    }
    return "error"; // Unhandled expression type
}


void MiniCompiler::semanticAnalysis(ASTNode* node) {
    if (!node) return;

    if (node->nodeType == "Program" || node->nodeType == "Block") {
        for (const auto& child : node->children) {
            semanticAnalysis(child.get());
        }
    }
    else if (node->nodeType == "Declaration") {
        std::string varName = node->value;
        if (symbolExists(varName)) {
            addError("Variable '" + varName + "' already declared.");
        } else {
            // Determine type from initializer
            if (!node->children.empty()) {
                std::string exprType = semanticAnalyzeExpression(node->children[0].get());
                if (exprType != "error") {
                    addSymbol(varName, exprType);
                    node->dataType = exprType; // Store type in the declaration node itself
                } else {
                    addSymbol(varName, "unknown"); // Mark as unknown if expr has error
                }
            } else {
                // Default type for declarations without initializer (e.g., var x;)
                // Assuming integer for now if no initializer, or could be "unknown"
                addSymbol(varName, "int");
                node->dataType = "int";
            }
        }
    }
    else if (node->nodeType == "Assignment") {
        std::string varName = node->value;
        if (!symbolExists(varName)) {
            addError("Undeclared variable '" + varName + "' in assignment.");
            return;
        }
        std::string varType = getSymbolType(varName);
        std::string exprType = semanticAnalyzeExpression(node->children[0].get());

        if (varType == "error" || exprType == "error") {
            return; // Errors already reported by semanticAnalyzeExpression
        }
        if (varType != exprType) {
            addError("Type mismatch in assignment for '" + varName + "': Expected " + varType + ", got " + exprType);
        }
    }
    else if (node->nodeType == "Print") {
        std::string exprType = semanticAnalyzeExpression(node->children[0].get());
        if (exprType == "error") {
            return; // Error already reported
        }
        // No specific type checking for print, as it can take int or string
    }
    else if (node->nodeType == "If" || node->nodeType == "IfElse") {
        std::string condType = semanticAnalyzeExpression(node->children[0].get());
        if (condType == "error") return;
        if (condType != "int") { // Condition should evaluate to a boolean (represented as int)
            addError("If condition must evaluate to an integer (boolean), found " + condType);
        }
        semanticAnalysis(node->children[1].get()); // Analyze if block
        if (node->nodeType == "IfElse") {
            semanticAnalysis(node->children[2].get()); // Analyze else block
        }
    }
    else if (node->nodeType == "Return") {
        semanticAnalyzeExpression(node->children[0].get()); // Just analyze, no specific return type check for now
    }
    // No need to recurse for NumberLiteral, StringLiteral, Identifier, BinaryExpr
    // as they are handled by semanticAnalyzeExpression which is called where needed.
}


std::string MiniCompiler::generateIntermediateCode(ASTNode* node, int& tempCount, int& labelCount) {
    if (!node) return "";

    if (node->nodeType == "Program" || node->nodeType == "Block") {
        for (const auto& child : node->children) {
            generateIntermediateCode(child.get(), tempCount, labelCount);
        }
        return "";
    }
    else if (node->nodeType == "Declaration") {
        if (!node->children.empty()) {
            std::string exprResult = generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
            addIntermediateCode(node->value + " = " + exprResult);
        } else {
            // Default initialization for integers, no default for strings
            if (node->dataType == "int") {
                addIntermediateCode(node->value + " = 0");
            }
        }
        return "";
    }
    else if (node->nodeType == "Assignment") {
        std::string exprResult = generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        addIntermediateCode(node->value + " = " + exprResult);
        return "";
    }
    else if (node->nodeType == "NumberLiteral" || node->nodeType == "StringLiteral" || node->nodeType == "Identifier") {
        return node->value; // Return the literal value or identifier name
    }
    else if (node->nodeType == "BinaryExpr") {
        std::string leftResult = generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        std::string rightResult = generateIntermediateCode(node->children[1].get(), tempCount, labelCount);

        std::string resultTemp = "t" + std::to_string(tempCount++);
        addIntermediateCode(resultTemp + " = " +
                            leftResult + " " +
                            node->value + " " + // Operator
                            rightResult);
        node->value = resultTemp; // Store the temp variable name for parent nodes
        return resultTemp;
    }
    else if (node->nodeType == "Print") {
        std::string exprResult = generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        if (node->children[0]->dataType == "string") {
            addIntermediateCode("print_string " + exprResult);
        } else {
            addIntermediateCode("print_int " + exprResult);
        }
        return "";
    }
    else if (node->nodeType == "Return") {
        std::string exprResult = generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        addIntermediateCode("return " + exprResult);
        return "";
    }
    else if (node->nodeType == "If") {
        std::string condResult = generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        std::string labelEnd = "L" + std::to_string(labelCount++);
        addIntermediateCode("ifFalse " + condResult + " goto " + labelEnd);
        generateIntermediateCode(node->children[1].get(), tempCount, labelCount); // If block
        addIntermediateCode(labelEnd + ":");
        return "";
    }
    else if (node->nodeType == "IfElse") {
        std::string condResult = generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        std::string labelElse = "L" + std::to_string(labelCount++);
        std::string labelEnd = "L" + std::to_string(labelCount++);
        addIntermediateCode("ifFalse " + condResult + " goto " + labelElse);
        generateIntermediateCode(node->children[1].get(), tempCount, labelCount); // If block
        addIntermediateCode("goto " + labelEnd);
        addIntermediateCode(labelElse + ":");
        generateIntermediateCode(node->children[2].get(), tempCount, labelCount); // Else block
        addIntermediateCode(labelEnd + ":");
        return "";
    }
    return "";
}


void MiniCompiler::generateAssembly(ASTNode* node, int& regCount) {
    if (!node) return;

    if (node->nodeType == "Program" || node->nodeType == "Block") {
        for (const auto& child : node->children) {
            generateAssembly(child.get(), regCount);
        }
    }
    else if (node->nodeType == "Declaration") {
        // If there's an initializer, generate code for it
        if (!node->children.empty()) {
            if (node->children[0]->dataType == "int") {
                if (node->children[0]->nodeType == "NumberLiteral") {
                    addAssemblyCode("mov dword [" + node->value + "], " + node->children[0]->value);
                } else { // Expression or Identifier
                    generateAssembly(node->children[0].get(), regCount); // Result will be in eax
                    addAssemblyCode("mov dword [" + node->value + "], eax");
                }
            } else if (node->children[0]->dataType == "string") {
                if (node->children[0]->nodeType == "StringLiteral") {
                    // For string literals, define them in .data section and assign their address
                    static int string_literal_count = 0;
                    std::string string_label = "str_lit_" + std::to_string(string_literal_count++);
                    addAssemblyCode("section .data"); // Temporarily switch to data section
                    addAssemblyCode(string_label + " db \"" + node->children[0]->value + "\", 0");
                    addAssemblyCode("section .text"); // Switch back
                    addAssemblyCode("mov dword [" + node->value + "], " + string_label); // Store address
                } else if (node->children[0]->nodeType == "Identifier") {
                    // Assigning one string variable to another
                    addAssemblyCode("mov dword eax, [" + node->children[0]->value + "]");
                    addAssemblyCode("mov dword [" + node->value + "], eax");
                }
            }
        } else {
            // Default initialization for variables without initializer
            if (node->dataType == "int") {
                addAssemblyCode("mov dword [" + node->value + "], 0");
            }
            // For string, might be null or empty string pointer, depending on language semantics
        }
    }
    else if (node->nodeType == "Assignment") {
        if (node->children[0]->dataType == "int") {
            if (node->children[0]->nodeType == "NumberLiteral") {
                addAssemblyCode("mov dword [" + node->value + "], " + node->children[0]->value);
            } else { // Expression or Identifier
                generateAssembly(node->children[0].get(), regCount); // Result in eax
                addAssemblyCode("mov dword [" + node->value + "], eax");
            }
        } else if (node->children[0]->dataType == "string") {
            if (node->children[0]->nodeType == "StringLiteral") {
                static int string_literal_count = 0;
                std::string string_label = "str_lit_assign_" + std::to_string(string_literal_count++);
                addAssemblyCode("section .data");
                addAssemblyCode(string_label + " db \"" + node->children[0]->value + "\", 0");
                addAssemblyCode("section .text");
                addAssemblyCode("mov dword [" + node->value + "], " + string_label);
            } else if (node->children[0]->nodeType == "Identifier") {
                addAssemblyCode("mov dword eax, [" + node->children[0]->value + "]");
                addAssemblyCode("mov dword [" + node->value + "], eax");
            }
        }
    }
    else if (node->nodeType == "NumberLiteral") {
        addAssemblyCode("mov eax, " + node->value);
    }
    else if (node->nodeType == "StringLiteral") {
        static int string_literal_count = 0;
        std::string string_label = "str_lit_direct_" + std::to_string(string_literal_count++);
        addAssemblyCode("section .data");
        addAssemblyCode(string_label + " db \"" + node->value + "\", 0");
        addAssemblyCode("section .text");
        addAssemblyCode("lea eax, [" + string_label + "]"); // Load address into eax
    }
    else if (node->nodeType == "Identifier") {
        if (getSymbolType(node->value) == "int") {
            addAssemblyCode("mov eax, dword [" + node->value + "]");
        } else if (getSymbolType(node->value) == "string") {
            addAssemblyCode("mov eax, dword [" + node->value + "]"); // Load string address
        }
    }
    else if (node->nodeType == "BinaryExpr") {
        generateAssembly(node->children[0].get(), regCount);
        addAssemblyCode("push eax");
        generateAssembly(node->children[1].get(), regCount);
        addAssemblyCode("mov ebx, eax");
        addAssemblyCode("pop eax");

        // Assuming only int-int operations for binary expressions for now
        if (node->value == "+") {
            addAssemblyCode("add eax, ebx");
        }
        else if (node->value == "-") {
            addAssemblyCode("sub eax, ebx");
        }
        else if (node->value == "*") {
            addAssemblyCode("imul eax, ebx");
        }
        else if (node->value == "/") {
            addAssemblyCode("cdq");
            addAssemblyCode("idiv ebx");
        }
        else if (node->value == "<") {
            addAssemblyCode("cmp eax, ebx");
            addAssemblyCode("setl al");
            addAssemblyCode("movzx eax, al");
        }
        else if (node->value == ">") {
            addAssemblyCode("cmp eax, ebx");
            addAssemblyCode("setg al");
            addAssemblyCode("movzx eax, al");
        }
        else if (node->value == "<=") {
            addAssemblyCode("cmp eax, ebx");
            addAssemblyCode("setle al");
            addAssemblyCode("movzx eax, al");
        }
        else if (node->value == ">=") {
            addAssemblyCode("cmp eax, ebx");
            addAssemblyCode("setge al");
            addAssemblyCode("movzx eax, al");
        }
        else if (node->value == "==") {
            addAssemblyCode("cmp eax, ebx");
            addAssemblyCode("sete al");
            addAssemblyCode("movzx eax, al");
        }
        else if (node->value == "!=") {
            addAssemblyCode("cmp eax, ebx");
            addAssemblyCode("setne al");
            addAssemblyCode("movzx eax, al");
        }
    }
    else if (node->nodeType == "Print") {
        generateAssembly(node->children[0].get(), regCount); // Value to print is in eax

        if (node->children[0]->dataType == "string") {
            addAssemblyCode("call print_string");
        } else { // Assuming int
            addAssemblyCode("call print_int");
        }
    }
    else if (node->nodeType == "Return") {
        generateAssembly(node->children[0].get(), regCount);
        addAssemblyCode("ret");
    }
    else if (node->nodeType == "If") {
        generateAssembly(node->children[0].get(), regCount);
        std::string labelEnd = "L" + std::to_string(regCount++);
        addAssemblyCode("cmp eax, 0");
        addAssemblyCode("je " + labelEnd); // If condition is false (eax is 0), jump to end
        generateAssembly(node->children[1].get(), regCount); // If block
        addAssemblyCode(labelEnd + ":");
    }
    else if (node->nodeType == "IfElse") {
        generateAssembly(node->children[0].get(), regCount);
        std::string labelElse = "L" + std::to_string(regCount++);
        std::string labelEnd = "L" + std::to_string(regCount++);
        addAssemblyCode("cmp eax, 0");
        addAssemblyCode("je " + labelElse); // If condition is false (eax is 0), jump to else
        generateAssembly(node->children[1].get(), regCount); // If block
        addAssemblyCode("jmp " + labelEnd);
        addAssemblyCode(labelElse + ":");
        generateAssembly(node->children[2].get(), regCount); // Else block
        addAssemblyCode(labelEnd + ":");
    }
}


void MiniCompiler::printTokens() const {
    for (const auto& token : tokens) {
        std::cout << "Line " << token.line << ", Column " << token.column << ": ";
        switch (token.type) {
            case TokenType::VAR: std::cout << "VAR"; break;
            case TokenType::PRINT: std::cout << "PRINT"; break;
            case TokenType::IF: std::cout << "IF"; break;
            case TokenType::ELSE: std::cout << "ELSE"; break;
            case TokenType::RETURN: std::cout << "RETURN"; break;
            case TokenType::ID: std::cout << "ID"; break;
            case TokenType::NUMBER: std::cout << "NUMBER"; break;
            case TokenType::STRING: std::cout << "STRING"; break;
            case TokenType::OP: std::cout << "OP"; break;
            case TokenType::COMPARE: std::cout << "COMPARE"; break;
            case TokenType::ASSIGN: std::cout << "ASSIGN"; break;
            case TokenType::LPAREN: std::cout << "LPAREN"; break;
            case TokenType::RPAREN: std::cout << "RPAREN"; break;
            case TokenType::LBRACE: std::cout << "LBRACE"; break;
            case TokenType::RBRACE: std::cout << "RBRACE"; break;
            case TokenType::SEMI: std::cout << "SEMI"; break;
            case TokenType::END: std::cout << "END"; break;
        }
        std::cout << " = " << token.value << std::endl;
    }
}

void MiniCompiler::printErrors() const {
    std::cout << "\nCompilation errors:" << std::endl;
    for (const auto& error : errors) {
        std::cout << error << std::endl;
    }
}

void MiniCompiler::printAST(const ASTNode* node) const {
    if (!node) return;
    std::cout << std::string(node->indentLevel * 2, ' ') << node->nodeType;
    if (!node->value.empty()) std::cout << " ('" << node->value << "')";
    if (!node->dataType.empty()) std::cout << " <" << node->dataType << ">";
    std::cout << std::endl;
    for (const auto& child : node->children) {
        printAST(child.get());
    }
}

void MiniCompiler::printASTJSON(const ASTNode* node, std::ostream& out, int indent) const {
    if (!node) return;
    out << std::string(indent, ' ') << "{\n";
    out << std::string(indent + 2, ' ') << "\"type\": \"" << node->nodeType << "\"";
    if (!node->value.empty()) {
        out << ",\n" << std::string(indent + 2, ' ') << "\"value\": \"" << node->value << "\"";
    }
    if (!node->dataType.empty()) {
        out << ",\n" << std::string(indent + 2, ' ') << "\"dataType\": \"" << node->dataType << "\"";
    }
    if (!node->children.empty()) {
        out << ",\n" << std::string(indent + 2, ' ') << "\"children\": [\n";
        for (size_t i = 0; i < node->children.size(); i++) {
            printASTJSON(node->children[i].get(), out, indent + 4);
            if (i + 1 < node->children.size()) out << ",\n";
        }
        out << "\n" << std::string(indent + 2, ' ') << "]";
    }
    out << "\n" << std::string(indent, ' ') << "}";
}

// Main function
int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <source_file.min>" << std::endl;
        return 1;
    }

    MiniCompiler compiler;
    compiler.compile(argv[1]);

    return compiler.hasErrors() ? 1 : 0;
}