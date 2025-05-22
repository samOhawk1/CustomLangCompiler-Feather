#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cctype>
#include <cstring>

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
    int indentLevel = 0;

    ASTNode(std::string type, std::string val = "")
        : nodeType(std::move(type)), value(std::move(val)) {}

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
    std::map<std::string, std::string> symbolTable;
    std::vector<std::string> errors;
    std::vector<std::string> intermediateCode;
    std::vector<std::string> assemblyCode;
    std::unique_ptr<ASTNode> ast;

    // Helper functions
    void addError(const std::string& error) { errors.push_back(error); }
    void addToken(TokenType type, const std::string& value, int line, int column) { tokens.emplace_back(type, value, line, column); }
    void addIntermediateCode(const std::string& code) { intermediateCode.push_back(code); }
    void addAssemblyCode(const std::string& code) { assemblyCode.push_back(code); }
    void addSymbol(const std::string& key, const std::string& value) { symbolTable[key] = value; }
    bool symbolExists(const std::string& key) const { return symbolTable.find(key) != symbolTable.end(); }

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
    void semanticAnalysis(ASTNode* node);

    // Intermediate Code Generation
    void generateIntermediateCode(ASTNode* node, int& tempCount, int& labelCount);

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
    generateAssembly(ast.get(), regCount);

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
                addError("Unterminated string literal");
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
            } else {
                value = source.substr(pos, 1);
                pos++;
                type = (value == "=") ? TokenType::ASSIGN : TokenType::COMPARE;
            }
        }
        else {
            switch (source[pos]) {
                case '=': type = TokenType::ASSIGN; value = source.substr(pos, 1); pos++; break;
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
    if (currentTokenIndex < tokens.size() - 1) {
        currentTokenIndex++;
    } else {
        currentTokenIndex = tokens.size();
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
        node = std::make_unique<ASTNode>("NumberLiteral", token.value);
        advance();
    }
    else if (token.type == TokenType::LPAREN) {
        advance();
        node = parseExpression(0);
        if (!node) return nullptr;
        if (currentToken().type != TokenType::RPAREN) {
            addError("Expected ')' after expression");
            return nullptr;
        }
        advance();
    }
    else {
        addError("Expected identifier, number, or '('");
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

        advance();
        auto right = parseExpression(prec + 1);
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
        addError("Expected identifier after 'var'");
        return nullptr;
    }

    std::string varName = currentToken().value;
    advance();
    std::unique_ptr<ASTNode> expr;

    if (currentToken().type == TokenType::ASSIGN) {
        advance();
        expr = parseExpression(0);
        if (!expr) {
            addError("Invalid expression in declaration");
            return nullptr;
        }
    }

    if (currentToken().type != TokenType::SEMI) {
        addError("Expected ';' at end of declaration");
        return nullptr;
    }
    advance();

    auto decl = std::make_unique<ASTNode>("Declaration", varName);
    if (expr) decl->addChild(std::move(expr));
    return decl;
}

std::unique_ptr<ASTNode> MiniCompiler::parsePrint() {
    advance();
    auto expr = parseExpression(0);
    if (!expr) {
        addError("Invalid expression in print");
        return nullptr;
    }

    if (currentToken().type != TokenType::SEMI) {
        addError("Expected ';' after print");
        return nullptr;
    }
    advance();

    auto node = std::make_unique<ASTNode>("Print");
    node->addChild(std::move(expr));
    return node;
}

std::unique_ptr<ASTNode> MiniCompiler::parseReturn() {
    advance();
    auto expr = parseExpression(0);
    if (!expr) {
        addError("Invalid expression in return");
        return nullptr;
    }
    if (currentToken().type != TokenType::SEMI) {
        addError("Expected ';' after 'return'");
        return nullptr;
    }
    advance();

    auto node = std::make_unique<ASTNode>("Return");
    node->addChild(std::move(expr));
    return node;
}

std::unique_ptr<ASTNode> MiniCompiler::parseIfElse() {
    advance();
    if (currentToken().type != TokenType::LPAREN) {
        addError("Expected '(' after 'if'");
        return nullptr;
    }
    advance();

    auto cond = parseExpression(0);
    if (!cond) {
        addError("Invalid condition in if");
        return nullptr;
    }
    if (currentToken().type != TokenType::RPAREN) {
        addError("Expected ')' after condition");
        return nullptr;
    }
    advance();

    if (currentToken().type != TokenType::LBRACE) {
        addError("Expected '{' after 'if' condition");
        return nullptr;
    }
    advance();

    auto ifBlock = parseProgramBlock();
    if (!ifBlock) return nullptr;

    if (currentToken().type == TokenType::ELSE) {
        advance();
        if (currentToken().type != TokenType::LBRACE) {
            addError("Expected '{' after 'else'");
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
            default:
                addError("Unexpected token in block");
                advance();
                break;
        }
    }
    if (currentToken().type == TokenType::RBRACE) advance();
    return block;
}

std::unique_ptr<ASTNode> MiniCompiler::parseProgram() {
    auto program = std::make_unique<ASTNode>("Program");
    while (currentToken().type != TokenType::END) {
        switch (currentToken().type) {
            case TokenType::VAR:
                if (auto decl = parseDeclaration()) program->addChild(std::move(decl));
                break;
            case TokenType::PRINT:
                if (auto prt = parsePrint()) program->addChild(std::move(prt));
                break;
            case TokenType::IF:
                if (auto ifstmt = parseIfElse()) program->addChild(std::move(ifstmt));
                break;
            case TokenType::RETURN:
                if (auto ret = parseReturn()) program->addChild(std::move(ret));
                break;
            default:
                addError("Unexpected token in program");
                advance();
                break;
        }
    }
    return program;
}

void MiniCompiler::semanticAnalysis(ASTNode* node) {
    if (!node) return;

    if (node->nodeType == "Program" || node->nodeType == "Block") {
        for (const auto& child : node->children) {
            semanticAnalysis(child.get());
        }
    }
    else if (node->nodeType == "Declaration") {
        if (symbolExists(node->value)) {
            addError("Variable '" + node->value + "' already declared.");
        } else {
            addSymbol(node->value, "variable");
        }
        if (!node->children.empty()) {
            semanticAnalysis(node->children[0].get());
        }
    }
    else if (node->nodeType == "Identifier") {
        if (!symbolExists(node->value)) {
            addError("Undeclared variable '" + node->value + "'");
        }
    }
    else if (node->nodeType == "BinaryExpr") {
        semanticAnalysis(node->children[0].get());
        semanticAnalysis(node->children[1].get());
    }
    else if (node->nodeType == "If" || node->nodeType == "IfElse") {
        for (const auto& child : node->children) {
            semanticAnalysis(child.get());
        }
    }
    else if (node->nodeType == "Print" || node->nodeType == "Return") {
        for (const auto& child : node->children) {
            semanticAnalysis(child.get());
        }
    }
}

void MiniCompiler::generateIntermediateCode(ASTNode* node, int& tempCount, int& labelCount) {
    if (!node) return;

    if (node->nodeType == "Program" || node->nodeType == "Block") {
        for (const auto& child : node->children) {
            generateIntermediateCode(child.get(), tempCount, labelCount);
        }
    }
    else if (node->nodeType == "Declaration") {
        if (!node->children.empty()) {
            generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
            addIntermediateCode(node->value + " = " + node->children[0]->value);
        } else {
            addIntermediateCode(node->value + " = 0");
        }
    }
    else if (node->nodeType == "NumberLiteral" || node->nodeType == "Identifier") {
        // value is already set
    }
    else if (node->nodeType == "BinaryExpr") {
        generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        generateIntermediateCode(node->children[1].get(), tempCount, labelCount);

        std::string resultTemp = "t" + std::to_string(tempCount++);
        addIntermediateCode(resultTemp + " = " +
                            node->children[0]->value + " " +
                            node->value + " " +
                            node->children[1]->value);
        node->value = resultTemp;
    }
    else if (node->nodeType == "Print") {
        generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        addIntermediateCode("print " + node->children[0]->value);
    }
    else if (node->nodeType == "Return") {
        generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        addIntermediateCode("return " + node->children[0]->value);
    }
    else if (node->nodeType == "If") {
        generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        std::string labelEnd = "L" + std::to_string(labelCount++);
        addIntermediateCode("ifFalse " + node->children[0]->value + " goto " + labelEnd);
        generateIntermediateCode(node->children[1].get(), tempCount, labelCount);
        addIntermediateCode(labelEnd + ":");
    }
    else if (node->nodeType == "IfElse") {
        generateIntermediateCode(node->children[0].get(), tempCount, labelCount);
        std::string labelElse = "L" + std::to_string(labelCount++);
        std::string labelEnd = "L" + std::to_string(labelCount++);
        addIntermediateCode("ifFalse " + node->children[0]->value + " goto " + labelElse);
        generateIntermediateCode(node->children[1].get(), tempCount, labelCount);
        addIntermediateCode("goto " + labelEnd);
        addIntermediateCode(labelElse + ":");
        generateIntermediateCode(node->children[2].get(), tempCount, labelCount);
        addIntermediateCode(labelEnd + ":");
    }
}

void MiniCompiler::generateAssembly(ASTNode* node, int& regCount) {
    if (!node) return;

    if (node->nodeType == "Program" || node->nodeType == "Block") {
        for (const auto& child : node->children) {
            generateAssembly(child.get(), regCount);
        }
    }
    else if (node->nodeType == "Declaration") {
        if (!node->children.empty()) {
            if (node->children[0]->nodeType == "NumberLiteral") {
                addAssemblyCode("mov [" + node->value + "], " + node->children[0]->value);
            }
            else if (node->children[0]->nodeType == "Identifier") {
                addAssemblyCode("mov [" + node->value + "], [" + node->children[0]->value + "]");
            }
            else if (node->children[0]->nodeType == "BinaryExpr") {
                generateAssembly(node->children[0].get(), regCount);
                addAssemblyCode("mov [" + node->value + "], eax");
            }
        } else {
            addAssemblyCode("mov [" + node->value + "], 0");
        }
    }
    else if (node->nodeType == "NumberLiteral") {
        addAssemblyCode("mov eax, " + node->value);
    }
    else if (node->nodeType == "Identifier") {
        addAssemblyCode("mov eax, [" + node->value + "]");
    }
    else if (node->nodeType == "BinaryExpr") {
        generateAssembly(node->children[0].get(), regCount);
        addAssemblyCode("push eax");
        generateAssembly(node->children[1].get(), regCount);
        addAssemblyCode("mov ebx, eax");
        addAssemblyCode("pop eax");

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
        generateAssembly(node->children[0].get(), regCount);
        addAssemblyCode("print eax");
    }
    else if (node->nodeType == "Return") {
        generateAssembly(node->children[0].get(), regCount);
        addAssemblyCode("ret");
    }
    else if (node->nodeType == "If") {
        generateAssembly(node->children[0].get(), regCount);
        std::string labelEnd = "L" + std::to_string(regCount++);
        addAssemblyCode("cmp eax, 0");
        addAssemblyCode("je " + labelEnd);
        generateAssembly(node->children[1].get(), regCount);
        addAssemblyCode(labelEnd + ":");
    }
    else if (node->nodeType == "IfElse") {
        generateAssembly(node->children[0].get(), regCount);
        std::string labelElse = "L" + std::to_string(regCount++);
        std::string labelEnd = "L" + std::to_string(regCount++);
        addAssemblyCode("cmp eax, 0");
        addAssemblyCode("je " + labelElse);
        generateAssembly(node->children[1].get(), regCount);
        addAssemblyCode("jmp " + labelEnd);
        addAssemblyCode(labelElse + ":");
        generateAssembly(node->children[2].get(), regCount);
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
    if (!node->value.empty()) std::cout << " (" << node->value << ")";
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
        std::cerr << "Usage: " << argv[0] << " insufficient number of agrumnets" << std::endl;
        return 1;
    }

    MiniCompiler compiler;
    compiler.compile(argv[1]);

    return compiler.hasErrors() ? 1 : 0;
}