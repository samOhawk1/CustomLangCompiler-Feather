#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

// Token types
typedef enum {
    VAR, PRINT, IF, ELSE, RETURN,
    ID, NUMBER, STRING,
    OP, COMPARE, ASSIGN,
    LPAREN, RPAREN, LBRACE, RBRACE, SEMI,
    END
} TokenType;

// Token structure
typedef struct {
    TokenType type;
    char* value;
    int line;
    int column;
} Token;

// AST Node structure
typedef struct ASTNode {
    char* nodeType;
    struct ASTNode** children;
    int childCount;
    char* value;
    int indentLevel;
} ASTNode;

// Compiler structure
typedef struct {
    Token* tokens;
    size_t tokenCount;
    size_t currentTokenIndex;
    char** symbolTableKeys;
    char** symbolTableValues;
    size_t symbolTableSize;
    char** errors;
    size_t errorCount;
    char** intermediateCode;
    size_t intermediateCodeSize;
    char** assemblyCode;         // Assembly code buffer
    size_t assemblyCodeSize;     // Assembly code buffer size
    ASTNode* ast;
} MiniCompiler;

// Helper strdup for portability
char* strdup_(const char* s) {
    size_t len = strlen(s) + 1;
    char* new = malloc(len);
    if (new == NULL) return NULL;
    memcpy(new, s, len);
    return new;
}

void addError(MiniCompiler* compiler, const char* error) {
    compiler->errors = realloc(compiler->errors, (compiler->errorCount + 1) * sizeof(char*));
    compiler->errors[compiler->errorCount++] = strdup_(error);
}

void addToken(MiniCompiler* compiler, TokenType type, const char* value, int line, int column) {
    compiler->tokens = realloc(compiler->tokens, (compiler->tokenCount + 1) * sizeof(Token));
    compiler->tokens[compiler->tokenCount].type = type;
    compiler->tokens[compiler->tokenCount].value = strdup_(value);
    compiler->tokens[compiler->tokenCount].line = line;
    compiler->tokens[compiler->tokenCount].column = column;
    compiler->tokenCount++;
}

ASTNode* createASTNode(const char* type, const char* val) {
    ASTNode* node = malloc(sizeof(ASTNode));
    node->nodeType = strdup_(type);
    node->value = val ? strdup_(val) : NULL;
    node->children = NULL;
    node->childCount = 0;
    node->indentLevel = 0;
    return node;
}

void addChild(ASTNode* parent, ASTNode* child) {
    parent->children = realloc(parent->children, (parent->childCount + 1) * sizeof(ASTNode*));
    child->indentLevel = parent->indentLevel + 1;
    parent->children[parent->childCount++] = child;
}

void addIntermediateCode(MiniCompiler* compiler, const char* code) {
    compiler->intermediateCode = realloc(compiler->intermediateCode, (compiler->intermediateCodeSize + 1) * sizeof(char*));
    compiler->intermediateCode[compiler->intermediateCodeSize++] = strdup_(code);
}

void addAssemblyCode(MiniCompiler* compiler, const char* code) {
    compiler->assemblyCode = realloc(compiler->assemblyCode, (compiler->assemblyCodeSize + 1) * sizeof(char*));
    compiler->assemblyCode[compiler->assemblyCodeSize++] = strdup_(code);
}

void addSymbol(MiniCompiler* compiler, const char* key, const char* value) {
    compiler->symbolTableKeys = realloc(compiler->symbolTableKeys, (compiler->symbolTableSize + 1) * sizeof(char*));
    compiler->symbolTableValues = realloc(compiler->symbolTableValues, (compiler->symbolTableSize + 1) * sizeof(char*));
    compiler->symbolTableKeys[compiler->symbolTableSize] = strdup_(key);
    compiler->symbolTableValues[compiler->symbolTableSize] = strdup_(value);
    compiler->symbolTableSize++;
}

bool symbolExists(MiniCompiler* compiler, const char* key) {
    for (size_t i = 0; i < compiler->symbolTableSize; i++) {
        if (strcmp(compiler->symbolTableKeys[i], key) == 0) {
            return true;
        }
    }
    return false;
}

void freeAST(ASTNode* node) {
    if (!node) return;
    free(node->nodeType);
    if (node->value) free(node->value);
    for (int i = 0; i < node->childCount; i++) {
        freeAST(node->children[i]);
    }
    if (node->children) free(node->children);
    free(node);
}

// Tokenizer
void tokenize(MiniCompiler* compiler, const char* source) {
    size_t pos = 0;
    int line = 1;
    int lineStart = 0;
    size_t sourceLen = strlen(source);

    while (pos < sourceLen) {
        while (pos < sourceLen && isspace(source[pos])) {
            if (source[pos] == '\n') {
                line++;
                lineStart = pos + 1;
            }
            pos++;
        }
        if (pos >= sourceLen) break;

        int tokenStart = pos;
        TokenType type = END;
        char* value = NULL;

        if (isalpha(source[pos]) || source[pos] == '_') {
            while (pos < sourceLen && (isalnum(source[pos]) || source[pos] == '_')) pos++;
            size_t len = pos - tokenStart;
            value = malloc(len + 1);
            strncpy(value, source + tokenStart, len);
            value[len] = '\0';
            if (strcmp(value, "var") == 0) type = VAR;
            else if (strcmp(value, "print") == 0) type = PRINT;
            else if (strcmp(value, "if") == 0) type = IF;
            else if (strcmp(value, "else") == 0) type = ELSE;
            else if (strcmp(value, "return") == 0) type = RETURN;
            else type = ID;
        }
        else if (isdigit(source[pos])) {
            while (pos < sourceLen && isdigit(source[pos])) pos++;
            size_t len = pos - tokenStart;
            value = malloc(len + 1);
            strncpy(value, source + tokenStart, len);
            value[len] = '\0';
            type = NUMBER;
        }
        else if (source[pos] == '"') {
            pos++;
            tokenStart = pos;
            while (pos < sourceLen && source[pos] != '"') pos++;
            if (pos >= sourceLen) {
                addError(compiler, "Unterminated string literal");
                break;
            }
            size_t len = pos - tokenStart;
            value = malloc(len + 1);
            strncpy(value, source + tokenStart, len);
            value[len] = '\0';
            type = STRING;
            pos++;
        }
        else if (strchr("+-*/", source[pos])) {
            value = malloc(2);
            value[0] = source[pos++];
            value[1] = '\0';
            type = OP;
        }
        else if (strchr("=!<>", source[pos])) {
            if (pos + 1 < sourceLen && source[pos+1] == '=') {
                value = malloc(3);
                value[0] = source[pos++];
                value[1] = source[pos++];
                value[2] = '\0';
                type = COMPARE;
            } else {
                value = malloc(2);
                value[0] = source[pos++];
                value[1] = '\0';
                if (value[0] == '=') type = ASSIGN;
                else type = COMPARE;
            }
        }
        else {
            switch (source[pos]) {
                case '=': type = ASSIGN; break;
                case '(': type = LPAREN; break;
                case ')': type = RPAREN; break;
                case '{': type = LBRACE; break;
                case '}': type = RBRACE; break;
                case ';': type = SEMI; break;
                default: {
                    char error[100];
                    snprintf(error, sizeof(error), 
                            "Illegal character '\\x%02x' at line %d, column %d",
                            (unsigned char)source[pos], line, pos - lineStart);
                    addError(compiler, error);
                    pos++;
                    continue;
                }
            }
            value = malloc(2);
            value[0] = source[pos++];
            value[1] = '\0';
        }

        addToken(compiler, type, value, line, tokenStart - lineStart);
        free(value);
    }
    addToken(compiler, END, "", line, 0);
}

// Parser
Token currentToken(MiniCompiler* compiler) {
    if (compiler->currentTokenIndex >= compiler->tokenCount) {
        Token end = {END, "", 0, 0};
        return end;
    }
    return compiler->tokens[compiler->currentTokenIndex];
}
void advance(MiniCompiler* compiler) {
    if (compiler->currentTokenIndex < compiler->tokenCount - 1) {
        compiler->currentTokenIndex++;
    } else {
        compiler->currentTokenIndex = compiler->tokenCount;
    }
}
int getPrecedence(const char* op) {
    if (op == NULL) return -1;
    if (strcmp(op, "+") == 0 || strcmp(op, "-") == 0) return 1;
    if (strcmp(op, "*") == 0 || strcmp(op, "/") == 0) return 2;
    if (strcmp(op, "<") == 0 || strcmp(op, ">") == 0 || 
        strcmp(op, "<=") == 0 || strcmp(op, ">=") == 0 ||
        strcmp(op, "==") == 0 || strcmp(op, "!=") == 0) return 0;
    return -1;
}

ASTNode* parseExpression(MiniCompiler* compiler, int minPrec);
ASTNode* parsePrimary(MiniCompiler* compiler) {
    Token token = currentToken(compiler);
    ASTNode* node = NULL;
    if (token.type == ID) {
        node = createASTNode("Identifier", token.value);
        advance(compiler);
    }
    else if (token.type == NUMBER) {
        node = createASTNode("NumberLiteral", token.value);
        advance(compiler);
    }
    else if (token.type == LPAREN) {
        advance(compiler);
        node = parseExpression(compiler, 0);
        if (!node) return NULL;
        if (currentToken(compiler).type != RPAREN) {
            addError(compiler, "Expected ')' after expression");
            freeAST(node);
            return NULL;
        }
        advance(compiler);
    }
    else {
        addError(compiler, "Expected identifier, number, or '('");
        return NULL;
    }
    return node;
}
ASTNode* parseExpression(MiniCompiler* compiler, int minPrec) {
    ASTNode* left = parsePrimary(compiler);
    if (!left) return NULL;
    while (true) {
        Token token = currentToken(compiler);
        if (token.type != OP && token.type != COMPARE) break;
        const char* op = token.value;
        int prec = getPrecedence(op);
        if (prec < minPrec) break;
        advance(compiler);
        ASTNode* right = parseExpression(compiler, prec + 1);
        if (!right) {
            freeAST(left);
            return NULL;
        }
        ASTNode* bin = createASTNode("BinaryExpr", op);
        addChild(bin, left);
        addChild(bin, right);
        left = bin;
    }
    return left;
}
ASTNode* parseDeclaration(MiniCompiler* compiler) {
    advance(compiler); // skip 'var'
    if (currentToken(compiler).type != ID) {
        addError(compiler, "Expected identifier after 'var'");
        return NULL;
    }
    char* varName = strdup_(currentToken(compiler).value);
    advance(compiler);
    ASTNode* expr = NULL;
    if (currentToken(compiler).type == ASSIGN) {
        advance(compiler);
        expr = parseExpression(compiler, 0);
        if (!expr) {
            addError(compiler, "Invalid expression in declaration");
            free(varName);
            return NULL;
        }
    }
    if (currentToken(compiler).type != SEMI) {
        addError(compiler, "Expected ';' at end of declaration");
        free(varName);
        return NULL;
    }
    advance(compiler);
    ASTNode* decl = createASTNode("Declaration", varName);
    free(varName);
    if (expr) addChild(decl, expr);
    return decl;
}
ASTNode* parsePrint(MiniCompiler* compiler) {
    advance(compiler);
    ASTNode* expr = parseExpression(compiler, 0);
    if (!expr) {
        addError(compiler, "Invalid expression in print");
        return NULL;
    }
    if (currentToken(compiler).type != SEMI) {
        addError(compiler, "Expected ';' after print");
        return NULL;
    }
    advance(compiler);
    ASTNode* node = createASTNode("Print", NULL);
    addChild(node, expr);
    return node;
}
ASTNode* parseReturn(MiniCompiler* compiler) {
    advance(compiler);
    ASTNode* expr = parseExpression(compiler, 0);
    if (currentToken(compiler).type != SEMI) {
        addError(compiler, "Expected ';' after 'return'");
        return NULL;
    }
    advance(compiler);
    ASTNode* node = createASTNode("Return", NULL);
    addChild(node, expr);
    return node;
}
ASTNode* parseProgramBlock(MiniCompiler* compiler);
ASTNode* parseIfElse(MiniCompiler* compiler) {
    advance(compiler);
    if (currentToken(compiler).type != LPAREN) {
        addError(compiler, "Expected '(' after 'if'");
        return NULL;
    }
    advance(compiler);
    ASTNode* cond = parseExpression(compiler, 0);
    if (currentToken(compiler).type != RPAREN) {
        addError(compiler, "Expected ')' after condition");
        return NULL;
    }
    advance(compiler);
    if (currentToken(compiler).type != LBRACE) {
        addError(compiler, "Expected '{' after 'if' condition");
        return NULL;
    }
    advance(compiler);
    ASTNode* ifBlock = parseProgramBlock(compiler);
    if (currentToken(compiler).type == ELSE) {
        advance(compiler);
        if (currentToken(compiler).type != LBRACE) {
            addError(compiler, "Expected '{' after 'else'");
            return NULL;
        }
        advance(compiler);
        ASTNode* elseBlock = parseProgramBlock(compiler);
        ASTNode* ifNode = createASTNode("IfElse", NULL);
        addChild(ifNode, cond);
        addChild(ifNode, ifBlock);
        addChild(ifNode, elseBlock);
        return ifNode;
    } else {
        ASTNode* ifNode = createASTNode("If", NULL);
        addChild(ifNode, cond);
        addChild(ifNode, ifBlock);
        return ifNode;
    }
}
ASTNode* parseProgramBlock(MiniCompiler* compiler) {
    ASTNode* block = createASTNode("Block", NULL);
    while (currentToken(compiler).type != RBRACE && currentToken(compiler).type != END) {
        switch (currentToken(compiler).type) {
            case VAR: addChild(block, parseDeclaration(compiler)); break;
            case PRINT: addChild(block, parsePrint(compiler)); break;
            case IF: addChild(block, parseIfElse(compiler)); break;
            case RETURN: addChild(block, parseReturn(compiler)); break;
            default: addError(compiler, "Unexpected token in block"); advance(compiler); break;
        }
    }
    if (currentToken(compiler).type == RBRACE) advance(compiler);
    return block;
}
ASTNode* parseProgram(MiniCompiler* compiler) {
    ASTNode* program = createASTNode("Program", NULL);
    while (currentToken(compiler).type != END) {
        switch (currentToken(compiler).type) {
            case VAR: { ASTNode* decl = parseDeclaration(compiler); if (decl) addChild(program, decl); break; }
            case PRINT: { ASTNode* prt = parsePrint(compiler); if (prt) addChild(program, prt); break; }
            case IF: { ASTNode* ifstmt = parseIfElse(compiler); if (ifstmt) addChild(program, ifstmt); break; }
            case RETURN: { ASTNode* ret = parseReturn(compiler); if (ret) addChild(program, ret); break; }
            default: addError(compiler, "Unexpected token in program"); advance(compiler); break;
        }
    }
    return program;
}

// Semantic Analysis
void semanticAnalysis(MiniCompiler* compiler, ASTNode* node) {
    if (!node) return;
    if (strcmp(node->nodeType, "Program") == 0) {
        for (int i = 0; i < node->childCount; i++)
            semanticAnalysis(compiler, node->children[i]);
    }
    else if (strcmp(node->nodeType, "Declaration") == 0) {
        if (symbolExists(compiler, node->value)) {
            char error[100];
            snprintf(error, sizeof(error), "Variable '%s' already declared.", node->value);
            addError(compiler, error);
        } else {
            addSymbol(compiler, node->value, "variable");
        }
        if (node->childCount > 0)
            semanticAnalysis(compiler, node->children[0]);
    }
    else if (strcmp(node->nodeType, "Identifier") == 0) {
        if (!symbolExists(compiler, node->value)) {
            char error[100];
            snprintf(error, sizeof(error), "Undeclared variable '%s'", node->value);
            addError(compiler, error);
        }
    }
    else if (strcmp(node->nodeType, "BinaryExpr") == 0) {
        semanticAnalysis(compiler, node->children[0]);
        semanticAnalysis(compiler, node->children[1]);
    }
}

// Intermediate Code Generation
void generateIntermediateCode(ASTNode* node, MiniCompiler* compiler, int* tempCount, int* labelCount) {
    if (!node) return;
    if (strcmp(node->nodeType, "Program") == 0) {
        for (int i = 0; i < node->childCount; i++)
            generateIntermediateCode(node->children[i], compiler, tempCount, labelCount);
    }
    else if (strcmp(node->nodeType, "Declaration") == 0) {
        if (node->childCount > 0) {
            generateIntermediateCode(node->children[0], compiler, tempCount, labelCount);
            char code[100];
            snprintf(code, sizeof(code), "%s = %s", node->value, node->children[0]->value);
            addIntermediateCode(compiler, code);
        } else {
            char code[100];
            snprintf(code, sizeof(code), "%s = 0", node->value);
            addIntermediateCode(compiler, code);
        }
    }
    else if (strcmp(node->nodeType, "BinaryExpr") == 0) {
        generateIntermediateCode(node->children[0], compiler, tempCount, labelCount);
        generateIntermediateCode(node->children[1], compiler, tempCount, labelCount);
        char resultTemp[20];
        snprintf(resultTemp, sizeof(resultTemp), "t%d", (*tempCount)++);
        char code[100];
        snprintf(code, sizeof(code), "%s = %s %s %s", 
                resultTemp, node->children[0]->value, node->value, node->children[1]->value);
        addIntermediateCode(compiler, code);
        free(node->value);
        node->value = strdup_(resultTemp);
    }
    else if (strcmp(node->nodeType, "Print") == 0) {
        generateIntermediateCode(node->children[0], compiler, tempCount, labelCount);
        char code[100];
        snprintf(code, sizeof(code), "print %s", node->children[0]->value);
        addIntermediateCode(compiler, code);
    }
    else if (strcmp(node->nodeType, "Return") == 0) {
        generateIntermediateCode(node->children[0], compiler, tempCount, labelCount);
        char code[100];
        snprintf(code, sizeof(code), "return %s", node->children[0]->value);
        addIntermediateCode(compiler, code);
    }
    else if (strcmp(node->nodeType, "If") == 0) {
        generateIntermediateCode(node->children[0], compiler, tempCount, labelCount);
        char labelEnd[20];
        snprintf(labelEnd, sizeof(labelEnd), "L%d", (*labelCount)++);
        char code[100];
        snprintf(code, sizeof(code), "ifFalse %s goto %s", node->children[0]->value, labelEnd);
        addIntermediateCode(compiler, code);
        generateIntermediateCode(node->children[1], compiler, tempCount, labelCount);
        snprintf(code, sizeof(code), "%s:", labelEnd);
        addIntermediateCode(compiler, code);
    }
    else if (strcmp(node->nodeType, "IfElse") == 0) {
        generateIntermediateCode(node->children[0], compiler, tempCount, labelCount);
        char labelElse[20], labelEnd[20];
        snprintf(labelElse, sizeof(labelElse), "L%d", (*labelCount)++);
        snprintf(labelEnd, sizeof(labelEnd), "L%d", (*labelCount)++);
        char code[100];
        snprintf(code, sizeof(code), "ifFalse %s goto %s", node->children[0]->value, labelElse);
        addIntermediateCode(compiler, code);
        generateIntermediateCode(node->children[1], compiler, tempCount, labelCount);
        snprintf(code, sizeof(code), "goto %s", labelEnd);
        addIntermediateCode(compiler, code);
        snprintf(code, sizeof(code), "%s:", labelElse);
        addIntermediateCode(compiler, code);
        generateIntermediateCode(node->children[2], compiler, tempCount, labelCount);
        snprintf(code, sizeof(code), "%s:", labelEnd);
        addIntermediateCode(compiler, code);
    }
    else if (strcmp(node->nodeType, "Block") == 0) {
        for (int i = 0; i < node->childCount; i++)
            generateIntermediateCode(node->children[i], compiler, tempCount, labelCount);
    }
}

// Assembly Code Generation
void generateAssembly(ASTNode* node, MiniCompiler* compiler, int* regCount) {
    if (!node) return;
    if (strcmp(node->nodeType, "Program") == 0) {
        for (int i = 0; i < node->childCount; i++)
            generateAssembly(node->children[i], compiler, regCount);
    }
    else if (strcmp(node->nodeType, "Declaration") == 0) {
        if (node->childCount > 0) {
            if (strcmp(node->children[0]->nodeType, "NumberLiteral") == 0) {
                char asmCode[100];
                snprintf(asmCode, sizeof(asmCode), "mov [%s], %s", node->value, node->children[0]->value);
                addAssemblyCode(compiler, asmCode);
            } else if (strcmp(node->children[0]->nodeType, "Identifier") == 0) {
                char asmCode[100];
                snprintf(asmCode, sizeof(asmCode), "mov [%s], [%s]", node->value, node->children[0]->value);
                addAssemblyCode(compiler, asmCode);
            } else if (strcmp(node->children[0]->nodeType, "BinaryExpr") == 0) {
                generateAssembly(node->children[0], compiler, regCount);
                char asmCode[100];
                snprintf(asmCode, sizeof(asmCode), "mov [%s], eax", node->value);
                addAssemblyCode(compiler, asmCode);
            }
        } else {
            char asmCode[100];
            snprintf(asmCode, sizeof(asmCode), "mov [%s], 0", node->value);
            addAssemblyCode(compiler, asmCode);
        }
    }
    else if (strcmp(node->nodeType, "NumberLiteral") == 0) {
        char asmCode[100];
        snprintf(asmCode, sizeof(asmCode), "mov eax, %s", node->value);
        addAssemblyCode(compiler, asmCode);
    }
    else if (strcmp(node->nodeType, "Identifier") == 0) {
        char asmCode[100];
        snprintf(asmCode, sizeof(asmCode), "mov eax, [%s]", node->value);
        addAssemblyCode(compiler, asmCode);
    }
    else if (strcmp(node->nodeType, "BinaryExpr") == 0) {
        generateAssembly(node->children[0], compiler, regCount);
        addAssemblyCode(compiler, "push eax");
        generateAssembly(node->children[1], compiler, regCount);
        addAssemblyCode(compiler, "mov ebx, eax");
        addAssemblyCode(compiler, "pop eax");
        char asmCode[100];
        if (strcmp(node->value, "+") == 0) {
            snprintf(asmCode, sizeof(asmCode), "add eax, ebx");
        } else if (strcmp(node->value, "-") == 0) {
            snprintf(asmCode, sizeof(asmCode), "sub eax, ebx");
        } else if (strcmp(node->value, "*") == 0) {
            snprintf(asmCode, sizeof(asmCode), "imul eax, ebx");
        } else if (strcmp(node->value, "/") == 0) {
            addAssemblyCode(compiler, "cdq");
            snprintf(asmCode, sizeof(asmCode), "idiv ebx");
        } else if (strcmp(node->value, "<") == 0) {
            snprintf(asmCode, sizeof(asmCode), "cmp eax, ebx");
            addAssemblyCode(compiler, asmCode);
            addAssemblyCode(compiler, "setl al");
            addAssemblyCode(compiler, "movzx eax, al");
            return;
        }
        addAssemblyCode(compiler, asmCode);
    }
    else if (strcmp(node->nodeType, "Print") == 0) {
        generateAssembly(node->children[0], compiler, regCount);
        addAssemblyCode(compiler, "print eax");
    }
    else if (strcmp(node->nodeType, "Return") == 0) {
        generateAssembly(node->children[0], compiler, regCount);
        addAssemblyCode(compiler, "ret");
    }
    else if (strcmp(node->nodeType, "If") == 0) {
        generateAssembly(node->children[0], compiler, regCount);
        char labelEnd[20];
        snprintf(labelEnd, sizeof(labelEnd), "L%d", (*regCount)++);
        char asmCode[100];
        snprintf(asmCode, sizeof(asmCode), "cmp eax, 0");
        addAssemblyCode(compiler, asmCode);
        snprintf(asmCode, sizeof(asmCode), "je %s", labelEnd);
        addAssemblyCode(compiler, asmCode);
        generateAssembly(node->children[1], compiler, regCount);
        snprintf(asmCode, sizeof(asmCode), "%s:", labelEnd);
        addAssemblyCode(compiler, asmCode);
    }
    else if (strcmp(node->nodeType, "IfElse") == 0) {
        generateAssembly(node->children[0], compiler, regCount);
        char labelElse[20], labelEnd[20];
        snprintf(labelElse, sizeof(labelElse), "L%d", (*regCount)++);
        snprintf(labelEnd, sizeof(labelEnd), "L%d", (*regCount)++);
        char asmCode[100];
        snprintf(asmCode, sizeof(asmCode), "cmp eax, 0");
        addAssemblyCode(compiler, asmCode);
        snprintf(asmCode, sizeof(asmCode), "je %s", labelElse);
        addAssemblyCode(compiler, asmCode);
        generateAssembly(node->children[1], compiler, regCount);
        snprintf(asmCode, sizeof(asmCode), "jmp %s", labelEnd);
        addAssemblyCode(compiler, asmCode);
        snprintf(asmCode, sizeof(asmCode), "%s:", labelElse);
        addAssemblyCode(compiler, asmCode);
        generateAssembly(node->children[2], compiler, regCount);
        snprintf(asmCode, sizeof(asmCode), "%s:", labelEnd);
        addAssemblyCode(compiler, asmCode);
    }
    else if (strcmp(node->nodeType, "Block") == 0) {
        for (int i = 0; i < node->childCount; i++)
            generateAssembly(node->children[i], compiler, regCount);
    }
}

// Print functions
void printTokens(MiniCompiler* compiler) {
    for (size_t i = 0; i < compiler->tokenCount; i++) {
        Token token = compiler->tokens[i];
        printf("Line %d, Column %d: ", token.line, token.column);
        switch (token.type) {
            case VAR: printf("VAR"); break;
            case PRINT: printf("PRINT"); break;
            case IF: printf("IF"); break;
            case ELSE: printf("ELSE"); break;
            case RETURN: printf("RETURN"); break;
            case ID: printf("ID"); break;
            case NUMBER: printf("NUMBER"); break;
            case STRING: printf("STRING"); break;
            case OP: printf("OP"); break;
            case COMPARE: printf("COMPARE"); break;
            case ASSIGN: printf("ASSIGN"); break;
            case LPAREN: printf("LPAREN"); break;
            case RPAREN: printf("RPAREN"); break;
            case LBRACE: printf("LBRACE"); break;
            case RBRACE: printf("RBRACE"); break;
            case SEMI: printf("SEMI"); break;
            case END: printf("END"); break;
        }
        printf(" = %s\n", token.value);
    }
}
void printErrors(MiniCompiler* compiler) {
    printf("\nCompilation errors:\n");
    for (size_t i = 0; i < compiler->errorCount; i++)
        printf("%s\n", compiler->errors[i]);
}
void printAST(ASTNode* node) {
    if (!node) return;
    printf("%*s%s", node->indentLevel * 2, "", node->nodeType);
    if (node->value) printf(" (%s)", node->value);
    printf("\n");
    for (int i = 0; i < node->childCount; i++)
        printAST(node->children[i]);
}
void printASTJSON(ASTNode* node, FILE* out, int indent) {
    if (!node) return;
    fprintf(out, "%*s{\n", indent, "");
    fprintf(out, "%*s  \"type\": \"%s\"", indent, "", node->nodeType);
    if (node->value) fprintf(out, ",\n%*s  \"value\": \"%s\"", indent, "", node->value);
    if (node->childCount > 0) {
        fprintf(out, ",\n%*s  \"children\": [\n", indent, "");
        for (int i = 0; i < node->childCount; i++) {
            printASTJSON(node->children[i], out, indent + 4);
            if (i + 1 < node->childCount) fprintf(out, ",\n");
        }
        fprintf(out, "\n%*s  ]", indent, "");
    }
    fprintf(out, "\n%*s}", indent, "");
}

// Main compilation function
void compile(MiniCompiler* compiler, const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Error: Could not open file '%s'\n", filename);
        return;
    }
    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);
    char* sourceCode = malloc(length + 1);
    fread(sourceCode, 1, length, file);
    sourceCode[length] = '\0';
    fclose(file);

    printf("=== Source Code ===\n");
    printf("%s\n\n", sourceCode);

    compiler->tokens = NULL;
    compiler->tokenCount = 0;
    compiler->currentTokenIndex = 0;
    compiler->symbolTableKeys = NULL;
    compiler->symbolTableValues = NULL;
    compiler->symbolTableSize = 0;
    compiler->errors = NULL;
    compiler->errorCount = 0;
    compiler->intermediateCode = NULL;
    compiler->intermediateCodeSize = 0;
    compiler->assemblyCode = NULL;
    compiler->assemblyCodeSize = 0;
    compiler->ast = NULL;

    printf("=== Lexical Analysis (Tokenization) ===\n");
    tokenize(compiler, sourceCode);
    printTokens(compiler);

    if (compiler->errorCount > 0) {
        printErrors(compiler);
        free(sourceCode);
        return;
    }

    printf("\n=== Syntax Analysis (Parsing) ===\n");
    compiler->ast = parseProgram(compiler);

    if (compiler->errorCount > 0) {
        printErrors(compiler);
        free(sourceCode);
        return;
    }

    printf("\nParse Tree:\n");
    if (compiler->ast) printAST(compiler->ast);

    printf("\nParse Tree (JSON):\n");
    if (compiler->ast) printASTJSON(compiler->ast, stdout, 0);
    printf("\n");

    printf("\n=== Semantic Analysis ===\n");
    semanticAnalysis(compiler, compiler->ast);

    if (compiler->errorCount > 0) {
        printErrors(compiler);
        free(sourceCode);
        return;
    }

    printf("\nSymbol Table:\n");
    for (size_t i = 0; i < compiler->symbolTableSize; i++)
        printf("%s: %s\n", compiler->symbolTableKeys[i], compiler->symbolTableValues[i]);

    printf("\n=== Intermediate Code Generation ===\n");
    int tempCount = 0;
    int labelCount = 0;
    generateIntermediateCode(compiler->ast, compiler, &tempCount, &labelCount);

    printf("\nIntermediate Code (Three-Address Code):\n");
    for (size_t i = 0; i < compiler->intermediateCodeSize; i++)
        printf("%zu: %s\n", i, compiler->intermediateCode[i]);

    printf("\n=== Assembly Code Generation ===\n");
    int regCount = 0;
    generateAssembly(compiler->ast, compiler, &regCount);

    for (size_t i = 0; i < compiler->assemblyCodeSize; i++)
        printf("%s\n", compiler->assemblyCode[i]);

    printf("\nCompilation successful!\n");
    free(sourceCode);
}

// Main
int main(int argc, char* argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <filename.mini>\n", argv[0]);
        return 1;
    }

    MiniCompiler compiler = {0};
    compile(&compiler, argv[1]);

    for (size_t i = 0; i < compiler.tokenCount; i++)
        free(compiler.tokens[i].value);
    free(compiler.tokens);

    for (size_t i = 0; i < compiler.symbolTableSize; i++) {
        free(compiler.symbolTableKeys[i]);
        free(compiler.symbolTableValues[i]);
    }
    free(compiler.symbolTableKeys);
    free(compiler.symbolTableValues);

    for (size_t i = 0; i < compiler.errorCount; i++)
        free(compiler.errors[i]);
    free(compiler.errors);

    for (size_t i = 0; i < compiler.intermediateCodeSize; i++)
        free(compiler.intermediateCode[i]);
    free(compiler.intermediateCode);#!/bin/bash

gcc tush.c -o tush && ./tush goel.mini
#!/bin/bash

gcc tush.c -o tush && ./tush goel.mini


    for (size_t i = 0; i < compiler.assemblyCodeSize; i++)
        free(compiler.assemblyCode[i]);
    free(compiler.assemblyCode);

    if (compiler.ast) freeAST(compiler.ast);

    return compiler.errorCount > 0 ? 1 : 0;
}