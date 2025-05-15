// 井字棋机器人控制系统 - 基于Arduino Mega2560
// 功能：支持人机对弈、棋盘旋转适应、棋子位置检测及纠正

// 引脚定义
// 霍尔传感器引脚(3x3棋盘，每个格子一个传感器)
const int hallSensorS[9] = {22, 24, 26, 28, 30, 32, 34, 36, 38}; // 南极检测引脚(黑棋)
const int hallSensorN[9] = {23, 25, 27, 29, 31, 33, 35, 37, 39}; // 北极检测引脚(白棋)
const int rotationSensor = 40; // 棋盘旋转检测传感器

// 步进电机控制引脚
const int X_STEP_PIN = 2;
const int X_DIR_PIN = 3;
const int X_ENABLE_PIN = 4;
const int Y_STEP_PIN = 5;
const int Y_DIR_PIN = 6;
const int Y_ENABLE_PIN = 7;

// 电磁铁控制引脚
const int MAGNET_PIN1 = 8;
const int MAGNET_PIN2 = 9;

// 物理参数设置
const int GRID_SIZE = 30; // 棋盘格尺寸(mm)
const int LINE_WIDTH = 2; // 棋盘线宽(mm)
const int STEPS_PER_MM = 80; // 步进电机每毫米的步数
const int MOTOR_SPEED = 1000; // 电机速度(微秒/步)

// 棋子放置区位置(相对于原点的偏移量，单位mm)
const int BLACK_PIECES_X = -(3*GRID_SIZE + LINE_WIDTH); // 棋盘左外侧
const int WHITE_PIECES_X = 3*GRID_SIZE + LINE_WIDTH; // 棋盘右外侧
const int PIECES_Y_START = -30; // 棋子放置区起始Y坐标
const int PIECES_GAP = 10; // 棋子之间的间隔

// 棋盘坐标，以中心点为原点
const int gridPositionsX[3] = {-GRID_SIZE, 0, GRID_SIZE};
const int gridPositionsY[3] = {-GRID_SIZE, 0, GRID_SIZE};

// 棋盘旋转时的位置修正
const int rotatedGridPositionsX[9] = {-42, 0, 42, -42, 0, 42, -42, 0, 42};
const int rotatedGridPositionsY[9] = {-42, -42, -42, 0, 0, 0, 42, 42, 42};

// 游戏状态变量
int boardState[9] = {0}; // 0:空, 1:黑棋, 2:白棋
int currentPlayer = 1; // 1:黑棋(人), 2:白棋(机器)
bool gameActive = false;
bool boardRotated = false;
int blackPiecesCount = 5;
int whitePiecesCount = 5;

void setup() {
  Serial.begin(9600);
  Serial.println("井字棋机器人初始化中...");
  
  // 初始化霍尔传感器引脚
  for (int i = 0; i < 9; i++) {
    pinMode(hallSensorS[i], INPUT);
    pinMode(hallSensorN[i], INPUT);
  }
  pinMode(rotationSensor, INPUT);
  
  // 初始化步进电机控制引脚
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  
  // 初始化电磁铁控制引脚
  pinMode(MAGNET_PIN1, OUTPUT);
  pinMode(MAGNET_PIN2, OUTPUT);
  
  // 使能电机(低电平有效)
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  
  // 停用电磁铁
  disableMagnet();
  
  Serial.println("初始化完成，请将机械臂移动到原点后开始操作");
  delay(1000);
}

void loop() {
  // 检测棋盘是否旋转
  boardRotated = digitalRead(rotationSensor) == HIGH;
  
  // 检测棋盘状态
  updateBoardState();
  
  // 检查是否有棋子被移动
  checkForMovedPieces();

  // 从串口读取命令
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    executeCommand(cmd);
  }
}

// 解析并执行命令
void executeCommand(char cmd) {
  switch(cmd) {
    case '1': // 任务1：放置黑棋到5号方格(中心位置)
      placeBlackToCenter();
      break;
    case '2': // 任务2：依次放置2黑2白到指定位置
      placeTwoBlackTwoWhite();
      break;
    case '3': // 任务3：旋转测试 - 依次放置2黑2白
      placeTwoBlackTwoWhiteWithRotation();
      break;
    case '4': // 任务4：机器执黑先行
      startGameAsMachine();
      break;
    case '5': // 任务5：机器执白后行
      startGameAsHuman();
      break;
    case '6': // 任务6：检测棋子移动
      enableMoveDetection();
      break;
    case 'r': // 重置游戏
      resetGame();
      break;
    case 'h': // 帮助
      printHelp();
      break;
  }
}

// 更新棋盘状态（读取所有霍尔传感器）
void updateBoardState() {
  int newState[9] = {0};
  
  for (int i = 0; i < 9; i++) {
    if (digitalRead(hallSensorS[i]) == HIGH) {
      newState[i] = 1; // 检测到黑棋(南极)
    } else if (digitalRead(hallSensorN[i]) == HIGH) {
      newState[i] = 2; // 检测到白棋(北极)
    } else {
      newState[i] = 0; // 没有棋子
    }
  }
  
  // 更新棋盘状态
  for (int i = 0; i < 9; i++) {
    boardState[i] = newState[i];
  }
}

// 检查棋子是否被移动(与存储的状态比较)
void checkForMovedPieces() {
  static int storedBoardState[9] = {0};
  static bool moveDetectionActive = false;
  
  if (!moveDetectionActive) return;
  
  updateBoardState();
  
  for (int i = 0; i < 9; i++) {
    if (boardState[i] != storedBoardState[i]) {
      Serial.print("检测到棋子被移动，位置:");
      Serial.println(i);
      
      // 恢复棋子位置
      if (storedBoardState[i] == 1) {
        moveToGrid(i);
        pickBlackPiece();
        placePiece();
        Serial.println("已将黑棋放回原位");
      } else if (storedBoardState[i] == 2) {
        moveToGrid(i);
        pickWhitePiece();
        placePiece();
        Serial.println("已将白棋放回原位");
      }
      
      // 更新存储的状态
      updateBoardState();
      for (int j = 0; j < 9; j++) {
        storedBoardState[j] = boardState[j];
      }
    }
  }
}

// 将机械臂移动到指定棋盘格
void moveToGrid(int gridNum) {
  int row = gridNum / 3;
  int col = gridNum % 3;
  int targetX, targetY;
  
  if (boardRotated) {
    // 旋转状态下使用调整后的坐标
    targetX = rotatedGridPositionsX[gridNum];
    targetY = rotatedGridPositionsY[gridNum];
  } else {
    // 正常状态下的坐标
    targetX = gridPositionsX[col];
    targetY = gridPositionsY[row];
  }
  
  moveToPosition(targetX, targetY);
}

// 控制步进电机移动到绝对位置(单位:mm)
void moveToPosition(int x, int y) {
  static int currentX = 0;
  static int currentY = 0;
  
  // 计算步数
  int stepsX = (x - currentX) * STEPS_PER_MM;
  int stepsY = (y - currentY) * STEPS_PER_MM;
  
  // 设置X轴方向
  if (stepsX > 0) {
    digitalWrite(X_DIR_PIN, HIGH);
  } else {
    digitalWrite(X_DIR_PIN, LOW);
    stepsX = -stepsX;
  }
  
  // 设置Y轴方向
  if (stepsY > 0) {
    digitalWrite(Y_DIR_PIN, HIGH);
  } else {
    digitalWrite(Y_DIR_PIN, LOW);
    stepsY = -stepsY;
  }
  
  // 移动X轴
  for (int i = 0; i < stepsX; i++) {
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  
  // 移动Y轴
  for (int i = 0; i < stepsY; i++) {
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(MOTOR_SPEED);
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(MOTOR_SPEED);
  }
  
  // 更新当前位置
  currentX = x;
  currentY = y;
  
  delay(200); // 稳定延时
}

// 移动到黑棋放置区并拾取一个黑棋
void pickBlackPiece() {
  int pieceIndex = 5 - blackPiecesCount;
  if (pieceIndex < 0 || pieceIndex >= 5) {
    Serial.println("错误: 黑棋已用完");
    return;
  }
  
  int y = PIECES_Y_START + pieceIndex * (GRID_SIZE + PIECES_GAP);
  moveToPosition(BLACK_PIECES_X, y);
  
  // 启动电磁铁吸取黑棋(南极)
  enableMagnetForBlack();
  delay(500); // 给电磁铁时间吸取棋子
  
  blackPiecesCount--;
}

// 移动到白棋放置区并拾取一个白棋
void pickWhitePiece() {
  int pieceIndex = 5 - whitePiecesCount;
  if (pieceIndex < 0 || pieceIndex >= 5) {
    Serial.println("错误: 白棋已用完");
    return;
  }
  
  int y = PIECES_Y_START + pieceIndex * (GRID_SIZE + PIECES_GAP);
  moveToPosition(WHITE_PIECES_X, y);
  
  // 启动电磁铁吸取白棋(北极)
  enableMagnetForWhite();
  delay(500); // 给电磁铁时间吸取棋子
  
  whitePiecesCount--;
}

// 放置当前拿着的棋子
void placePiece() {
  // 停用电磁铁，放下棋子
  disableMagnet();
  delay(500); // 给足够时间放下棋子
}

// 启用电磁铁吸取黑棋(南极)
void enableMagnetForBlack() {
  digitalWrite(MAGNET_PIN1, HIGH);
  digitalWrite(MAGNET_PIN2, LOW);
}

// 启用电磁铁吸取白棋(北极)
void enableMagnetForWhite() {
  digitalWrite(MAGNET_PIN1, LOW);
  digitalWrite(MAGNET_PIN2, HIGH);
}

// 停用电磁铁
void disableMagnet() {
  digitalWrite(MAGNET_PIN1, LOW);
  digitalWrite(MAGNET_PIN2, LOW);
}

// 任务1: 将黑棋放到中心位置(5号方格)
void placeBlackToCenter() {
  Serial.println("执行任务1: 将黑棋放置到中心位置");
  
  pickBlackPiece();
  moveToGrid(4); // 5号方格(索引从0开始，所以是4)
  placePiece();
  
  Serial.println("任务1完成");
}

// 任务2: 依次放置2黑2白到指定位置
void placeTwoBlackTwoWhite() {
  Serial.println("执行任务2: 依次放置2黑2白到指定位置");
  
  int positions[4];
  Serial.println("请依次输入4个放置位置(1-9):");
  
  for (int i = 0; i < 4; i++) {
    while (!Serial.available()) {
      delay(100);
    }
    
    int pos = Serial.parseInt();
    if (pos >= 1 && pos <= 9) {
      positions[i] = pos - 1; // 转换为0-8索引
    } else {
      Serial.println("无效位置，请输入1-9之间的数字");
      i--; // 重试
    }
  }
  
  // 放置2黑2白
  for (int i = 0; i < 4; i++) {
    if (i < 2) {
      pickBlackPiece();
    } else {
      pickWhitePiece();
    }
    
    moveToGrid(positions[i]);
    placePiece();
    Serial.print("放置棋子到位置: ");
    Serial.println(positions[i] + 1);
    delay(1000);
  }
  
  Serial.println("任务2完成");
}

// 任务3: 旋转测试 - 依次放置2黑2白
void placeTwoBlackTwoWhiteWithRotation() {
  Serial.println("执行任务3: 旋转测试 - 依次放置2黑2白");
  Serial.println("请先旋转棋盘45度，然后继续...");
  
  while (!boardRotated) {
    boardRotated = digitalRead(rotationSensor) == HIGH;
    delay(100);
  }
  
  Serial.println("检测到棋盘旋转，开始任务");
  placeTwoBlackTwoWhite(); // 复用任务2的代码
}

// 任务4: 机器执黑先行
void startGameAsMachine() {
  Serial.println("执行任务4: 机器执黑先行");
  resetGame();
  currentPlayer = 1; // 机器执黑
  gameActive = true;
  
  // 询问第一步
  Serial.println("请指定机器第一步棋的位置(1-9):");
  while (!Serial.available()) {
    delay(100);
  }
  int firstMove = Serial.parseInt() - 1;
  
  if (firstMove >= 0 && firstMove < 9) {
    // 机器第一步
    pickBlackPiece();
    moveToGrid(firstMove);
    placePiece();
    boardState[firstMove] = 1;
    printBoard();
    
    // 等待人类白棋
    Serial.println("请放置您的白棋...");
    bool validMove = false;
    int humanMove = -1;
    
    while (!validMove) {
      delay(500);
      updateBoardState();
      
      for (int i = 0; i < 9; i++) {
        if (boardState[i] == 2 && i != humanMove) { // 发现新的白棋
          humanMove = i;
          validMove = true;
          break;
        }
      }
    }
    
    Serial.print("检测到人类落子在位置: ");
    Serial.println(humanMove + 1);
    
    // 判断人类是否走错
    if (shouldWin(firstMove, humanMove)) {
      Serial.println("检测到人类走棋错误，机器将获胜");
      int winningMove = findWinningMove(1);
      
      if (winningMove != -1) {
        pickBlackPiece();
        moveToGrid(winningMove);
        placePiece();
        boardState[winningMove] = 1;
        printBoard();
        Serial.println("机器获胜！");
      }
    } else {
      Serial.println("人类走棋正确，继续游戏...");
      // 这里可以继续实现游戏逻辑
    }
  } else {
    Serial.println("无效的位置，请重新开始游戏");
  }
}

// 判断是否应该赢得游戏(针对任务4)
bool shouldWin(int machineMove, int humanMove) {
  // 简单策略：如果人类没有挡住机器可能的连线，则机器应该获胜
  
  // 中间为先手的必胜策略
  if (machineMove == 4) {
    // 如果人类没有下在角上，则机器可获胜
    return (humanMove != 0 && humanMove != 2 && humanMove != 6 && humanMove != 8);
  }
  
  // 角落为先手的必胜策略
  if (machineMove == 0 || machineMove == 2 || machineMove == 6 || machineMove == 8) {
    // 如果人类没有下在中间，则机器可获胜
    return (humanMove != 4);
  }
  
  // 边缘为先手的必胜策略较复杂，这里简化处理
  return false;
}

// 任务5: 机器执白后行
void startGameAsHuman() {
  Serial.println("执行任务5: 人类执黑先行，机器执白");
  resetGame();
  currentPlayer = 1; // 人类执黑
  gameActive = true;
  
  Serial.println("请放置您的黑棋...");
  
  while (gameActive) {
    // 等待人类放置黑棋
    int humanMove = waitForHumanMove(1);
    if (humanMove == -1) continue;
    
    Serial.print("检测到人类黑棋落在位置: ");
    Serial.println(humanMove + 1);
    
    // 检查游戏是否结束
    if (checkWin(1)) {
      Serial.println("人类获胜！");
      gameActive = false;
      break;
    }
    
    if (isBoardFull()) {
      Serial.println("游戏平局！");
      gameActive = false;
      break;
    }
    
    // 机器下白棋
    Serial.println("机器思考中...");
    delay(1000); // 模拟思考
    
    int machineMove = findBestMove(2);
    
    pickWhitePiece();
    moveToGrid(machineMove);
    placePiece();
    boardState[machineMove] = 2;
    
    Serial.print("机器白棋落在位置: ");
    Serial.println(machineMove + 1);
    printBoard();
    
    // 检查游戏是否结束
    if (checkWin(2)) {
      Serial.println("机器获胜！");
      gameActive = false;
      break;
    }
    
    if (isBoardFull()) {
      Serial.println("游戏平局！");
      gameActive = false;
      break;
    }
  }
}

// 等待人类下棋
int waitForHumanMove(int playerValue) {
  int previousState[9];
  for (int i = 0; i < 9; i++) {
    previousState[i] = boardState[i];
  }
  
  while (true) {
    delay(300);
    updateBoardState();
    
    for (int i = 0; i < 9; i++) {
      if (boardState[i] == playerValue && previousState[i] != playerValue) {
        return i;
      }
    }
  }
}

// 任务6: 启用棋子移动检测
void enableMoveDetection() {
  Serial.println("执行任务6: 启用棋子移动检测");
  
  // 存储当前棋盘状态
  updateBoardState();
  static int storedBoardState[9] = {0};
  for (int i = 0; i < 9; i++) {
    storedBoardState[i] = boardState[i];
  }
  
  Serial.println("棋子移动检测已启用，请尝试移动一颗棋子");
}

// 重置游戏状态
void resetGame() {
  for (int i = 0; i < 9; i++) {
    boardState[i] = 0;
  }
  blackPiecesCount = 5;
  whitePiecesCount = 5;
  currentPlayer = 1;
  gameActive = false;
  Serial.println("游戏已重置");
}

// 打印帮助信息
void printHelp() {
  Serial.println("井字棋机器人命令:");
  Serial.println("1: 放置黑棋到中心位置");
  Serial.println("2: 依次放置2黑2白到指定位置");
  Serial.println("3: 旋转测试 - 依次放置2黑2白");
  Serial.println("4: 机器执黑先行");
  Serial.println("5: 人类执黑先行");
  Serial.println("6: 启用棋子移动检测");
  Serial.println("r: 重置游戏");
  Serial.println("h: 显示此帮助信息");
}

// 打印当前棋盘状态
void printBoard() {
  Serial.println("当前棋盘状态:");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      int index = i * 3 + j;
      char symbol = ' ';
      if (boardState[index] == 1) symbol = 'X';
      if (boardState[index] == 2) symbol = 'O';
      Serial.print(symbol);
      if (j < 2) Serial.print('|');
    }
    Serial.println();
    if (i < 2) Serial.println("-+-+-");
  }
}

// 检查玩家是否获胜（已修正，增加了棋子数量判断）
bool checkWin(int player) {
  // 增加棋子数量判断
  if (player == 1 && blackPiecesCount > 2) return false;
  if (player == 2 && whitePiecesCount > 2) return false;
  
  // 检查行
  for (int i = 0; i < 3; i++) {
    if (boardState[i*3] == player && boardState[i*3+1] == player && boardState[i*3+2] == player) {
      return true;
    }
  }
  
  // 检查列
  for (int i = 0; i < 3; i++) {
    if (boardState[i] == player && boardState[i+3] == player && boardState[i+6] == player) {
      return true;
    }
  }
  
  // 检查对角线
  if (boardState[0] == player && boardState[4] == player && boardState[8] == player) {
    return true;
  }
  if (boardState[2] == player && boardState[4] == player && boardState[6] == player) {
    return true;
  }
  
  return false;
}

// 检查棋盘是否已满
bool isBoardFull() {
  for (int i = 0; i < 9; i++) {
    if (boardState[i] == 0) {
      return false;
    }
  }
  return true;
}

// 找出获胜位置
int findWinningMove(int player) {
  // 遍历所有位置，检查是否可以获胜
  for (int i = 0; i < 9; i++) {
    if (boardState[i] == 0) {
      // 尝试在此位置下棋
      boardState[i] = player;
      
      // 检查是否获胜
      if (checkWin(player)) {
        boardState[i] = 0; // 恢复
        return i;
      }
      
      boardState[i] = 0; // 恢复
    }
  }
  
  return -1; // 没有找到获胜位置
}

// 找出最佳下棋位置(极小化极大算法)
int findBestMove(int player) {
  int opponent = (player == 1) ? 2 : 1;
  int bestScore = -1000;
  int bestMove = -1;
  
  // 首先检查自己能否获胜
  int winningMove = findWinningMove(player);
  if (winningMove != -1) {
    return winningMove;
  }
  
  // 阻止对手获胜
  int blockingMove = findWinningMove(opponent);
  if (blockingMove != -1) {
    return blockingMove;
  }
  
  // 如果中心为空，优先选择中心
  if (boardState[4] == 0) {
    return 4;
  }
  
  // 然后尝试角落
  int corners[4] = {0, 2, 6, 8};
  for (int i = 0; i < 4; i++) {
    if (boardState[corners[i]] == 0) {
      return corners[i];
    }
  }
  
  // 最后尝试边缘
  int edges[4] = {1, 3, 5, 7};
  for (int i = 0; i < 4; i++) {
    if (boardState[edges[i]] == 0) {
      return edges[i];
    }
  }
  
  // 如果没有找到合适的位置(不应该发生)
  for (int i = 0; i < 9; i++) {
    if (boardState[i] == 0) {
      return i;
    }
  }
  
  return -1; // 没有可走的位置
}