/**
 * @brief Great Escape
 *
 * An experimental A.I. for the Great Escape challenge (www.codinggame.com).
 * Implement a non optimized short path algorithm and some kind a magic tuning.
 * No defensive strategy when placing walls (not enought time to try going in this direction).
 *
 * @author 12/02/2015 FBranca
*/
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <deque>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <memory>
#include <iomanip> // std::setw
#include <time.h>

using namespace std;

/// When a cell is not reachable or unitialized.
const float   _distanceUnreachable = std::numeric_limits<float>::infinity();

const int _max_players = 4;   //!< Maximum number of players

/// Direction
enum EDirection
{
    eDirLeft,
    eDirRight,
    eDirUp,
    eDirDown
};

/// Position (x,y) on the game board
struct Position
{
    int x;  //!< x coordinate (0 is left )
    int y;  //!< y coordinate (0 is top)

    /// Constructor
    Position(int aX, int aY) : x(aX), y(aY) {}

    /// Move in a direction
    inline Position Left  (void) const
    {
        return Position(x-1, y);
    }
    inline Position Right (void) const
    {
        return Position(x+1, y);
    }
    inline Position Up    (void) const
    {
        return Position(x, y-1);
    }
    inline Position Down  (void) const
    {
        return Position(x, y+1);
    }

    /// Move in a direction
    Position Move(EDirection aDir) const
    {
        Position ret(x,y);
        switch (aDir)
        {
        case eDirLeft  :
            ret.x --;
            break;
        case eDirRight :
            ret.x ++;
            break;
        case eDirUp    :
            ret.y --;
            break;
        case eDirDown  :
            ret.y ++;
            break;
        }
        return ret;
    }
};

/// Stringify - For debug purpose
ostream& operator<<(ostream& os, const Position& aPosition)
{
    os << "[" << aPosition.x << "," << aPosition.y << "]";
    return os;
}

typedef std::deque<Position> Path;

/**
 * @brief   Convert a Edirection to string
 *
 * @param[in]   adirection  Edirection to stringify
*/
std::string ToString (EDirection adirection)
{
    std::string Return;

    switch (adirection)
    {
    case eDirUp:
        Return = "UP";
        break;
    case eDirDown:
        Return = "DOWN";
        break;
    case eDirLeft:
        Return = "LEFT";
        break;
    case eDirRight:
        Return = "RIGHT";
        break;
    }

    return Return;
}


// deprecated, use Position instead
void Displace (int& aX, int& aY, EDirection adirection)
{
    switch (adirection)
    {
    case eDirLeft  :
        aX --;
        break;
    case eDirRight :
        aX ++;
        break;
    case eDirUp    :
        aY --;
        break;
    case eDirDown  :
        aY ++;
        break;
    }
}


/**
 * @brief   Time measurement Singleton
 *
 * Compute the time elapsed since the beginning of a turn.
*/
class CTimeSingleton
{
public:
    /// Start counting
    void Start (void)
    {
        mBegin = clock();
    }

    /// Return elapsed time (in ms) since last call to Start
    float Elapsed (void) const
    {
        return static_cast<float>(clock() - mBegin) / 1000;
    }

    /// Return Singleton Instance
    static CTimeSingleton& GetInstance (void)
    {
        static CTimeSingleton* pInstance = nullptr;

        if (!pInstance)
        {
            pInstance = new CTimeSingleton();
        }

        return *pInstance;
    }

private:
    /// Constructor (private to guaranty the Singleton)
    CTimeSingleton (void) :
        mBegin(clock())
    {
    }

    /// Copy Constructor (private to guaranty the Singleton)
    CTimeSingleton (const CTimeSingleton& aInstance);

private:
    clock_t mBegin; //!< Timestamp when measure started
};


/**
 * @brief Informations about a player
*/
class Player
{
public:
    Player (int aId);

    inline int GetId (void) const
    {
        return mId;
    }
    inline Position GetPosition     (void) const
    {
        return mPosition;
    }
    inline bool IsAlive         (void) const
    {
        return mbAlive;
    }
    inline int  GetWallsLeft    (void) const
    {
        return mWallsLeft;
    }
    inline EDirection GetDirection (void) const
    {
        return mdirection;
    }
    void Update (int aX, int aY, int aWallsLeft);
    inline void IncNbTimesBlocked (void)
    {
        mNbTimesBlocked ++;
    }
    inline int  GetNbTimesBlocked (void) const
    {
        return mNbTimesBlocked;
    }

private:
    int         mId;        //!< Player Id
    bool        mbAlive;    //!< Is the player still alive ?
    EDirection  mdirection; //!< Side the player have to reach
    Position    mPosition;  //!< Position of the Player
    int         mWallsLeft;      //!< Walls left
    int         mNbTimesBlocked; //!< Nb times we have blocked this player
};

Player::Player (int aId) :
    mId (aId),
    mbAlive (true),
    mPosition (-1,-1),
    mWallsLeft (0),
    mNbTimesBlocked (0)
{
    switch (mId)
    {
    case 0:
        mdirection = eDirRight;
        break;
    case 1:
        mdirection = eDirLeft;
        break;
    case 2:
        mdirection = eDirDown;
        break;
    case 3:
        mdirection = eDirUp;
        break; // Maybe one day ...
    default:
        // No more than 4 players
        assert (false);
        break;
    }
}

void Player::Update (int aX, int aY, int aWallsLeft)
{
    mPosition.x = aX;
    mPosition.y = aY;
    mWallsLeft = aWallsLeft;

    // -1 = player is dead
    mbAlive = (aWallsLeft >= 0);
}

/**
 * @brief   Result of an evaluation
*/
struct CMove
{
    float       currentDistance;
    EDirection  moveDirection;  //!< direction
    float       moveDistance;   //!< Distance left (lesser is better)

    CMove (void) :
        currentDistance (std::numeric_limits<float>::infinity()),
        moveDirection (eDirUp),
        moveDistance  (std::numeric_limits<float>::infinity())
    {}

    void KeepMin (float aDistance, EDirection aDirection)
    {
        // TODO _distanceUnreachable vs infinity
        if ((aDistance != _distanceUnreachable) && (aDistance < moveDistance))
        {
            moveDistance = aDistance;
            moveDirection = aDirection;
        }
    }

    /// When a player is dead
    void SetDead (void)
    {
        // worst distance possible
        currentDistance = std::numeric_limits<float>::infinity();
        moveDistance = std::numeric_limits<float>::infinity();
    }

    bool IsBlocked(void) const
    {
        return moveDistance == _distanceUnreachable;
    }
    bool IsWinner (void) const
    {
        return currentDistance <= 0.0;
    }
};

/// Stringify - For debug purpose
ostream& operator<<(ostream& os, const CMove& aMove)
{
    os << "move{" << std::setprecision(4) << aMove.currentDistance << ", " << aMove.moveDistance << " " << ToString(aMove.moveDirection) << "}";
    return os;
}

/**
 * @brief  List of players
*/
class PlayerList
{
public:
    /// List of player ids
    typedef std::vector<int> TIdVector;

public:
    PlayerList (size_t aCount, int aMyId);

    inline void UpdatePlayer (int aId, int aX, int aY, int aWallsLeft);

    // Return our player id
    int GetMyId (void) const
    {
        return mMyPlayerId;
    }

    int GetTargetId (void) const
    {
        return mTargetId;
    }

    void SetTargetId (int aId)
    {
        mTargetId = aId;
    }

    // Return number of players
    inline size_t Count (void) const
    {
        return mPlayers.size();
    }

    // Return number of players
    size_t CountAlive (void) const
    {
        size_t  count = mPlayers.size();
        size_t  alive = 0;
        for (int idx=0; idx<count; idx++)
        {
            if (mPlayers[idx].IsAlive())
            {
                alive ++;
            }
        }
        return alive;
    }

    // Access a player
    inline Player& operator [] (int aIndex)
    {
        return mPlayers[aIndex];
    }

    // Access a player (const)
    inline const Player& operator [] (int aIndex) const
    {
        return mPlayers[aIndex];
    }

    // Get a list of others players id
    inline const TIdVector& GetOthersIds (void) const
    {
        return mOthersId;
    }

private:
    std::vector<Player> mPlayers;       //!< Array of all players
    int                 mMyPlayerId;    //!< Our player ID
    int                 mTargetId;      //!< Id of the player to block
    TIdVector           mOthersId;      //!< Array containing the IDs of others players (not me)
};


/**
 * @brief Constructor
 *
 * @param[in] aCount    Number of players
 * @param[in] aMyId     My player ID
*/
PlayerList::PlayerList (size_t aCount, int aMyId) :
    mMyPlayerId (aMyId),
    mTargetId   (-1)
{
    int idx;

    for (idx = 0; idx < aCount; idx ++)
    {
        mPlayers.push_back(Player(idx));
        if (idx != aMyId)
        {
            mOthersId.push_back(idx);
        }
    }
}

/**
 * @brief   Update a player
 *
 * @param[in]   aId         Player ID to update
 * @param[in]   aX          X position of the player
 * @param[in]   aY          Y position of the player
 * @param[in]   aWallsLeft  Numbers of walls left
*/
void PlayerList::UpdatePlayer (int aId, int aX, int aY, int aWallsLeft)
{
    assert ((aId >= 0) && (aId < mPlayers.size()));
    Player& player = mPlayers[aId];

    player.Update (aX, aY, aWallsLeft);
    if (!player.IsAlive() && (aId == mTargetId))
    {
        // no more a target
        mTargetId = -1;
    }
}


/**
 * @brief   Matrix interface
 *
 * An algorithm can use this interface to evaluate
 * each cell of the game board
*/
class IMatrix
{
public:
    virtual ~IMatrix (void) {}

    virtual void  GetSize   (int& aWidth, int& aHeight) const = 0;
    virtual void  SetMatrix (const Position& aPosition, float aValue) = 0;
    virtual float GetMatrix (const Position& aPosition) const = 0;
};


/**
 * @brief Wall interface
 *
 * Abstraction from walls implementation.
 * Open the door to futur optimisations :)
*/
class IWalls
{
public:
    virtual ~IWalls (void) {}

    virtual void AddWall   (int aX, int aY, char aOrientation) = 0;
    virtual void DelWall   (int aX, int aY, char aOrientation) = 0;
    virtual bool IsBlocked (const Position& aPosition, EDirection adirection) const = 0;
    virtual bool Collide   (int aX, int aY, char aOrientation) const = 0;
};


/**
 * @brief   Hold informations about a cell
 *
 * All data concerning a cell are grouped here.
 * All scores for each players
*/
struct Cell
{
    // Default Constructor
    Cell (void) :
        mbBlocked (false)
    {
        for (int idx=0; idx<_max_players; idx++)
        {
            mPlayerDistances[idx] = _distanceUnreachable;
        }
    }

    float   mPlayerDistances[_max_players];  //!< score of each player for this cell
    bool    mbBlocked;                       //!< Is this cell blocked ?
};


/**
 * @brief  Game board
 *
 * Implements a representation of the game board.
 * This implementation includes cells and walls in the same structure.
*/
class Board : public IWalls
{
public:
    /**
     * @brief  Constructor
     *
     * @param[in] aWidth    Board width
     * @param[in] aHeight   Board height
    */
    Board (size_t aWidth, size_t aHeight) :
        mWidth  (aWidth),
        mHeight (aHeight),
        mInternalWidth  ((aWidth * 2) + 1),
        mInternalHeight ((aHeight * 2) + 1),
        mpCells (new Cell[mInternalWidth * mInternalHeight] )
    {
        for (int x = 0; x < mInternalWidth; x++)
        {
            GetInternalCell(x,0).mbBlocked = true;
            GetInternalCell(x,mInternalHeight-1).mbBlocked = true;
        }

        for (int y = 0; y < mInternalHeight; y++)
        {
            GetInternalCell(0,y).mbBlocked = true;
            GetInternalCell(mInternalWidth-1,y).mbBlocked = true;
        }
    }

    Board (const Board& aInstance) :
        mWidth  (aInstance.mWidth),
        mHeight (aInstance.mHeight),
        mInternalWidth  (aInstance.mInternalWidth),
        mInternalHeight (aInstance.mInternalHeight),
        mpCells (new Cell[mInternalWidth * mInternalHeight] )
    {
        for (int idx=0; idx<mInternalWidth * mInternalHeight; idx++)
        {
            mpCells[idx] = aInstance.mpCells[idx];
        }
    }

    virtual ~Board (void)
    {
        delete [] mpCells;
    }

    inline int GetWidth (void) const
    {
        return mWidth;
    }

    inline int GetHeight (void) const
    {
        return mHeight;
    }

    /**
     * @brief [IWall] Add a wall
     *
     * @param[in] aX Wall X position beginning
     * @param[in] aY Wall Y position beginning
     * @param[in] aOrientation  'V' for a vertical wall, 'H' for Horizontal (others values forbiden)
    */
    virtual void AddWall (int aX, int aY, char aOrientation)
    {
        SetWall (aX, aY, aOrientation, true);
    }

    /**
     * @brief [IWall] Remove a wall
     *
     * @param[in] aX Wall X position beginning
     * @param[in] aY Wall Y position beginning
     * @param[in] aOrientation  'V' for a vertical wall, 'H' for Horizontal (others values forbiden)
    */
    virtual void DelWall   (int aX, int aY, char aOrientation)
    {
        SetWall (aX, aY, aOrientation, false);
    }

    /**
     * @brief Add or remove a wall
     *
     * @param[in] aX Wall X position beginning
     * @param[in] aY Wall Y position beginning
     * @param[in] aOrientation  'V' for a vertical wall, 'H' for Horizontal (others values forbiden)
     * @param[in] abBlocked true to add a wall, false to remove it.
    */
    void SetWall   (int aX, int aY, char aOrientation, bool abBlocked)
    {
        int X = (aX * 2);
        int Y = (aY * 2);

        if (aOrientation == 'H')
        {
            for (int idx = 1; idx < 4; idx++)
            {
                Cell& cell = GetInternalCell(X+idx, Y);
                if (cell.mbBlocked != abBlocked)
                {
                    cell.mbBlocked = abBlocked;
                }
            }
        }
        if (aOrientation == 'V')
        {
            for (int idx = 1; idx < 4; idx++)
            {
                Cell& cell = GetInternalCell(X, Y+idx);
                if (cell.mbBlocked != abBlocked)
                {
                    cell.mbBlocked = abBlocked;
                }
            }
        }
    }

    /**
     * @brief  Test if a new wall would collide an existing one
     *
     * @param[in] aX    Beginning of the wall (x)
     * @param[in] aY    Beginning of the wall (y)
     * @param[in] aOrientation    'V' for a vertical wall, 'H' for horizontal
     *
     * @return true if the new wall would collide or is out of bounds
    */
    virtual bool Collide (int aX, int aY, char aOrientation) const
    {
        bool bRet = false;

        if ((aX >= 0) && (aY >= 0) && (aX < mWidth) && (aY < mHeight))
        {
            int  X = (aX * 2);
            int  Y = (aY * 2);

            if (aOrientation == 'H')
            {
                for (int Offset = 3; (Offset > 0) && ! bRet; Offset--)
                {
                    if (++X < mInternalWidth)
                    {
                        bRet = GetInternalCell(X, Y).mbBlocked;
                    }
                    else
                    {
                        bRet = true;
                    }
                }
            }
            if (aOrientation == 'V')
            {
                for (int Offset = 3; (Offset > 0) && ! bRet; Offset--)
                {
                    if (++Y < mInternalHeight)
                    {
                        bRet = GetInternalCell(X, Y).mbBlocked;
                    }
                    else
                    {
                        bRet = true;
                    }
                }
            }
        }
        else
        {
            bRet = true;
        }

        return bRet;
    }
    /**
     * @brief  Test if a move toward a direction is blocked by a wall
     *
     * @param[in]   aPosition  Current position
     * @param[in]   aDirection  Move direction
     *
     * @return true if a wall blocks the move
    */
    virtual bool IsBlocked (const Position& aPosition, EDirection aDirection) const
    {
        bool bBlocked;
        int x = (aPosition.x * 2) + 1;
        int y = (aPosition.y * 2) + 1;

        Displace (x, y, aDirection);

        if ((x >= 0) && (y >= 0) && (x < mInternalWidth) && (y < mInternalHeight))
        {
            bBlocked = GetInternalCell (x, y).mbBlocked;
        }
        else
        {
            bBlocked = true;
        }
        return bBlocked;
    }

    Cell& GetCell (const Position& aPosition)
    {
        assert(aPosition.x >= 0);
        assert(aPosition.x < 9);
        assert(aPosition.y >= 0);
        assert(aPosition.y < 9);
        return GetInternalCell ((aPosition.x * 2) + 1, (aPosition.y * 2) + 1);
    }

    const Cell& GetCell (const Position& aPosition) const
    {
        assert(aPosition.x >= 0);
        assert(aPosition.x < mWidth);
        assert(aPosition.y >= 0);
        assert(aPosition.y < mHeight);
        return GetInternalCell ((aPosition.x * 2) + 1, (aPosition.y * 2) + 1);
    }

    void PrintDbgDistance (int aPlayerId) const
    {
        std::cerr << "P" << aPlayerId << " distances" << std::endl;
        for (int y = 0; y < mHeight; y++)
        {
            for (int x = 0; x < mWidth; x++)
            {
                const Cell& cell = GetCell(Position(x, y));

                std::cerr << std::setw(8);
                if (cell.mPlayerDistances[aPlayerId] != _distanceUnreachable)
                {
                    std::cerr << std::setprecision(2) << cell.mPlayerDistances[aPlayerId];
                }
                else
                {
                    std::cerr << "+";
                }
            }
            std::cerr << std::endl;
        }
    }

public:
    Cell& GetInternalCell (int aX, int aY)
    {
        assert(aX >= 0);
        assert(aX < mInternalWidth);
        assert(aY >= 0);
        assert(aY < mInternalHeight);
        size_t  Offset = aX + (aY * mInternalWidth);
        return mpCells[Offset];
    }


    const Cell& GetInternalCell (int aX, int aY) const
    {
        assert(aX >= 0);
        assert(aX < mInternalWidth);
        assert(aY >= 0);
        assert(aY < mInternalHeight);
        size_t  Offset = aX + (aY * mInternalWidth);
        return mpCells[Offset];
    }

private:
    Board& operator = (const Board& aInstance);

private:
    int     mWidth;     //!< Board Width
    int     mHeight;    //!< Board Height
    int     mInternalWidth; //!< Internal width (Include Cells & Walls)
    int     mInternalHeight;//!< Internal height (Include Cells & Walls)
    Cell*   mpCells; //!< Arrays of cells & walls
};



/**
 * @brief   Evaluate each cell of the board
 *
 * Compute shortest path matrix.
*/
class BoardWeights
{
public:
    BoardWeights (const IWalls& aWalls, IMatrix& aScores);

    void  Update   (const Player& aPlayer);
    CMove BestMove (const Position& aPosition);
    Path  ShortestPath (const Position& aPosition)
    {
        Path        path;
        Position    position = aPosition;
        CMove       move;

        do
        {
            path.push_back(position);
            move = BestMove (position);
            position = position.Move(move.moveDirection);
        }
        while (!move.IsWinner());

        return path;
    }

private:
    void Init           (void);
    void InitDirection  (const Player& aPlayer);
    void UpdateCell     (const Position& aPosition, int aDepth=1);
    float MinNeighbors   (const Position& aPosition);

private:
    const IWalls& mWalls;       //!< Pointer to walls interface
    IMatrix&      mDistance;    //!< Pointer to the current distance matrix
    int           mWidth;       //!< Board width
    int           mHeight;      //!< Board height
};

BoardWeights::BoardWeights (const IWalls& aWalls, IMatrix& aScores) :
    mWalls (aWalls),
    mDistance (aScores),
    mWidth (0),
    mHeight (0)
{
    mDistance.GetSize(mWidth, mHeight);
}

void BoardWeights::Update (const Player& aPlayer)
{
    Init ();
    InitDirection (aPlayer);
}

/* too late...
void BoardWeights::PenalizeSides (const Player& aPlayer, float aCoef)
{
    Position pos(0,0);
    bool bVert = (aPlayer.GetDirection() == eDirUp) || (aPlayer.GetDirection() == eDirDown));
    bool bLoop = true;
    
    if (bVert)
    {
        float middle = mHeight/2;
        while (pos.y < mHeight)
        {
            float score = mDistance.GetMatrix(pos) * (aCoef * fabs(pos.y-middle));
            mDistance.SetMatrix(pos, score);
            float score = mDistance.GetMatrix(pos) * (aCoef * fabs(pos.y-middle));
            mDistance.SetMatrix(pos, score);
            pos.y++;
        }
    }
}
*/

void BoardWeights::Init (void)
{
    for (int x = 0; x < mWidth; x++)
    {
        for (int y = 0; y < mHeight; y++)
        {
            mDistance.SetMatrix (Position(x, y), _distanceUnreachable);
        }
    }
}

/**
 * @brief  choose the best move for a position
 * @param[in] aPosition Position to examine
 * @return Best move structure
*/
CMove BoardWeights::BestMove (const Position& aPosition)
{
    CMove   move; // return value

    move.currentDistance = mDistance.GetMatrix(aPosition);
    if ((aPosition.x > 0) && ! mWalls.IsBlocked(aPosition, eDirLeft))
    {
        move.KeepMin (mDistance.GetMatrix(aPosition.Left()), eDirLeft);
    }
    if ((aPosition.x < mWidth-1) && ! mWalls.IsBlocked(aPosition, eDirRight))
    {
        move.KeepMin (mDistance.GetMatrix(aPosition.Right()), eDirRight);
    }
    if ((aPosition.y > 0) && ! mWalls.IsBlocked(aPosition, eDirUp))
    {
        move.KeepMin (mDistance.GetMatrix(aPosition.Up()), eDirUp);
    }
    if ((aPosition.y < mHeight-1) && ! mWalls.IsBlocked(aPosition, eDirDown))
    {
        move.KeepMin (mDistance.GetMatrix(aPosition.Down()), eDirDown);
    }

    return move;
}

void BoardWeights::InitDirection (const Player& aPlayer)
{
    float initValue = 0.0; // experimental static_cast<float>(aPlayer.GetId()) * 0.1;

    switch (aPlayer.GetDirection())
    {
    case eDirUp:
    {
        for (int x = 0; x < mWidth; x ++)
        {
            mDistance.SetMatrix(Position(x, 0), initValue);
        }
        for (int x = 0; x < mWidth; x ++)
        {
            UpdateCell (Position(x, 1));
        }
    }
    break;

    case eDirDown:
    {
        int y = mHeight-1;
        for (int x = 0; x < mWidth; x ++)
        {
            mDistance.SetMatrix(Position(x, y), initValue);
        }
        for (int x = 0; x < mWidth; x ++)
        {
            UpdateCell (Position(x, y-1));
        }
    }
    break;

    case eDirLeft:
    {
        for (int y = 0; y < mHeight; y ++)
        {
            mDistance.SetMatrix(Position(0, y), initValue);
        }
        for (int y = 0; y < mHeight; y ++)
        {
            UpdateCell (Position(1, y));
        }
    }
    break;

    case eDirRight:
    {
        int x = mWidth-1;
        for (int y = 0; y < mHeight; y ++)
        {
            mDistance.SetMatrix(Position(x, y), initValue);
        }
        for (int y = 0; y < mHeight; y ++)
        {
            UpdateCell (Position(x-1, y));
        }
    }
    break;
    }
}


void BoardWeights::UpdateCell (const Position& aPosition, int aDepth)
{
    if ((aPosition.x >= 0) && (aPosition.x < mWidth) && (aPosition.y >= 0) && (aPosition.y < mHeight))
    {
        float distance = MinNeighbors(aPosition);
        if  (distance < std::numeric_limits<float>::infinity())
        {
            distance += 1.0;
            if (distance < mDistance.GetMatrix(aPosition))
            {
                mDistance.SetMatrix(aPosition, distance);

                UpdateCell (aPosition.Left(),  aDepth+1);
                UpdateCell (aPosition.Right(),  aDepth+1);
                UpdateCell (aPosition.Up(),  aDepth+1);
                UpdateCell (aPosition.Down(),  aDepth+1);
            }
        }
    }
}


void Min (float& aValue, float aNeighbor)
{
    if (aNeighbor < aValue)
    {
        aValue = aNeighbor;
    }
}

float BoardWeights::MinNeighbors (const Position& aPosition)
{
    float Return = std::numeric_limits<float>::infinity();

    if ((aPosition.x > 0) && ! mWalls.IsBlocked(aPosition, eDirLeft))
    {
        Min (Return, mDistance.GetMatrix(aPosition.Left()));
    }

    if ((aPosition.x < mWidth-1) && ! mWalls.IsBlocked(aPosition, eDirRight))
    {
        Min (Return, mDistance.GetMatrix(aPosition.Right()));
    }

    if ((aPosition.y > 0) && ! mWalls.IsBlocked(aPosition, eDirUp))
    {
        Min (Return, mDistance.GetMatrix(aPosition.Up()));
    }

    if ((aPosition.y < mHeight-1) && ! mWalls.IsBlocked(aPosition, eDirDown))
    {
        Min (Return, mDistance.GetMatrix(aPosition.Down()));
    }

    return Return;
}


/**
 * @brief   Access cell distance for a player id
*/
class PlayerDistanceMatrix : public IMatrix
{
public:
    PlayerDistanceMatrix (Board& aBoard, int aPlayerId) :
        mBoard    (aBoard),
        mPlayerId (aPlayerId)
    {
    }

    virtual void  GetSize (int& aWidth, int& aHeight) const
    {
        aWidth  = mBoard.GetWidth();
        aHeight = mBoard.GetHeight();
    }

    virtual void  SetMatrix (const Position& aPosition, float aScore)
    {
        mBoard.GetCell(aPosition).mPlayerDistances[mPlayerId] = aScore;
    }

    virtual float GetMatrix (const Position& aPosition) const
    {
        assert (mPlayerId >= 0);
        assert (mPlayerId < 3);
        return mBoard.GetCell(aPosition).mPlayerDistances[mPlayerId];
    }

private:
    Board&  mBoard;
    int     mPlayerId;
};


/**
 * @brief   Evaluation of a wall placement
*/
struct WallResult
{
    WallResult (void) :
        x (0),
        y (0),
        orientation(' '),
        distance (_distanceUnreachable),
        flightDistance (_distanceUnreachable),
        increase (0.0),
        myIncrease (0.0),
        score (0.0)
    {}

    WallResult (int aX, int aY, char aOrientation) :
        x (aX),
        y (aY),
        orientation(aOrientation),
        distance (_distanceUnreachable),
        flightDistance (_distanceUnreachable),
        increase (0.0),
        myIncrease (0.0),
        score (0.0)
    {}

    int     x;              //!< Wall origin, x coordinate
    int     y;              //!< Wall origin, y coordinate
    char    orientation;    //!< Orientation ('H'orizontal / 'V'ertical)
    float   distance;       //!< Distance increase if this wall is placed
    float   flightDistance; //!< Distance without any wall
    float   increase;       //!< Distance increase by placing the wall
    float   myIncrease;     //!< Distance increase for me if the wall is placed
    float   score;          //!< Score (Mix of all)
    std::string text;       //!< Text to display when placing the wall
};

/**
 * @brief  Result of an evaluation for all players
 *
 * - Build an evaluation of the shortest distance to goal for each player
 * - Detect if a player is blocked
 * - Can be printed on the debug output
*/
class ScoreBoard
{
    friend ostream& operator<<(ostream& os, const ScoreBoard& aSb);

public:
    ScoreBoard (void) :
        mbOnePlayerBlocked(false)
    {
    }

    void Build (const PlayerList& aPlayers, Board& aBoard)
    {
        mbOnePlayerBlocked = false;
        for (size_t idx=0; idx<aPlayers.Count(); idx++)
        {
            const Player&   player  = aPlayers[idx];
            if (player.IsAlive())
            {
                PlayerDistanceMatrix    distanceMatrix(aBoard, idx); // distance for player number 'idx'
                BoardWeights            board(aBoard, distanceMatrix);

                board.Update  (player);
                mPlayerBestMoves[idx] = board.BestMove(player.GetPosition());
                mbOnePlayerBlocked |= mPlayerBestMoves[idx].IsBlocked();
            }
            else
            {
                mPlayerBestMoves[idx].SetDead ();
            }
        }
    }

    void PrintDebug (const PlayerList& aPlayers, bool abEndl=true) const
    {
        for (size_t idx=0; idx<aPlayers.Count(); idx++)
        {
            const Player&   player  = aPlayers[idx];
            std::cerr << "P" << idx
                      << " " << player.GetPosition()
                      << (player.IsAlive() ? " alive " : " dead ") << mPlayerBestMoves[idx];
            if (abEndl)
            {
                std::cerr << std::endl;
            }
        }
    }

    const CMove& operator[] (int aIdx) const
    {
        return mPlayerBestMoves[aIdx];
    }

    inline bool OnePlayerBlocked (void) const
    {
        return mbOnePlayerBlocked;
    }

private:
    bool    mbOnePlayerBlocked;  //!< True if one player is blocked in this score board
    CMove   mPlayerBestMoves[4]; //!< Best moves for each player
};


/**
 * @brief   Wall placement intelligence
 *
 * Evaluate if wall placement seems a good idea.
 * Return the position of the wall to place.
 */
class WallPlacer
{
public:
    // Wall placement evaluation
    std::string PlaceWall1      (PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard);
    std::string PlaceWall       (PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard);

private:
    enum EBest
    {
        eBestScore,
        eBestDistance,
        eBestIncrease
    };
    EBest   mBestChoice;

    static float FlightDistance  (const Player& aPlayer, const Board& aBoard);
    void Best (WallResult& aResult, const WallResult& aChallenger);
    int         ShouldPlaceWall (PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard);
    WallResult  PlaceWallSimple (const PlayerList& aPlayers, int aPlayerIdToBlock, Board& aBoard, const ScoreBoard& aScoreBoard);
    WallResult  PlaceWallPath   (const PlayerList& aPlayers, int aPlayerIdToBlock, Board& aBoard, const ScoreBoard& aScoreBoard);
    int         FindTarget (const PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard);
    WallResult  TestWall (const PlayerList& aPlayers, int aPlayerId, const ScoreBoard& aScoreBoard, Board& aBoard, int aX, int aY, char aOrientation) const;

    bool Filter3P_Basic (const PlayerList& aPlayers) const
    {
        int  playerCount = aPlayers.Count();
        bool bShouldPlaceWall = true;
        if (playerCount > 2)
        {
            if (aPlayers.GetMyId() < 2)
            {
                bool bNoWall = true;
                for (int idx=0; idx<playerCount; idx++)
                {
                    bNoWall &= (aPlayers[idx].GetWallsLeft() == 6);
                }
                bShouldPlaceWall = ! bNoWall;
            }
        }
        return bShouldPlaceWall;
    }
};


/**
 * @brief   Compute 'flight distance' (distance without considering walls)
 *
 * @param[in]  aPlayer  Player to compute the distance for
 * @param[in]  aBoard   Game board
 *
 * @return  Number of moves needed to reach the aimed side
*/
float WallPlacer::FlightDistance  (const Player& aPlayer, const Board& aBoard)
{
    float distance;

    switch (aPlayer.GetDirection())
    {
    case eDirLeft:
        distance = 1 + aPlayer.GetPosition().x;
        break;
    case eDirRight:
        distance = aBoard.GetWidth() - aPlayer.GetPosition().x;
        break;
    case eDirUp:
        distance = 1 + aPlayer.GetPosition().y;
        break;
    case eDirDown:
        distance = aBoard.GetHeight()  + aPlayer.GetPosition().y;
        break;
    }

    return distance;
}

// Not used anymore .. old fashioned ;)
int WallPlacer::ShouldPlaceWall (PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard)
{
    int   target = -1;
    const Player& me = aPlayers[aPlayers.GetMyId()];

    if (    (me.GetWallsLeft() > 0)
            &&  Filter3P_Basic(aPlayers) )
    {
        if (aPlayers.GetTargetId() >= 0)
        {
            // Shortcut to the other player
            const Player&   otherPlayer = aPlayers[aPlayers.GetTargetId()];
            float           myDistance = aScoreBoard[aPlayers.GetMyId()].moveDistance;

            // We should not place a wall if we might be handicaped by it
            // Compare the distance of the other player on my distance matrix to my distance.
            float otherDistanceFromMyView = aBoard.GetCell (otherPlayer.GetPosition()).mPlayerDistances[aPlayers.GetMyId()];

            if ((otherDistanceFromMyView > myDistance) || (aScoreBoard[aPlayers.GetTargetId()].moveDistance < 3.0))
            {
                // Do place a wall if the other player have advantage
                if (aScoreBoard[aPlayers.GetTargetId()].moveDistance <= myDistance)
                {
                    target = aPlayers.GetTargetId();
                }
            }
        }
        else
        {
            target = FindTarget(aPlayers, aBoard, aScoreBoard);
            aPlayers.SetTargetId (target);
        }
    }

    return target;
}

// Not used anymore .. old fashioned ;)
int WallPlacer::FindTarget (const PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard)
{
    int   target = -1;
    const PlayerList::TIdVector&          OthersId = aPlayers.GetOthersIds();
    PlayerList::TIdVector::const_iterator iId;

    float MinDistance = 2.0;
    if (aPlayers.CountAlive() > 2)
    {
        MinDistance = 3.0;
    }

    // Iterate through others players ids
    for (iId = OthersId.begin(); iId != OthersId.end(); ++iId)
    {
        if (aScoreBoard[*iId].currentDistance < MinDistance)
        {
            target = *iId;
            MinDistance = aScoreBoard[*iId].currentDistance;
        }
    }

    return target;
}

/**
 * @brief   Evaluation of wall placement
 *
 * @param[in]     aPlayers    Player list
 * @param[in,out] aBoard      Game board
 * @param[in]     aScoreBoard Scoreboard - current evaluation of all players
 */
std::string WallPlacer::PlaceWall1 (PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard)
{
    // Wall selection is made on score
    mBestChoice = eBestScore;

    std::string     ret;
    int             idPlayerToBlock = ShouldPlaceWall (aPlayers, aBoard, aScoreBoard);
    WallResult      wallRes = PlaceWallPath(aPlayers, idPlayerToBlock, aBoard, aScoreBoard);

std::cerr << idPlayerToBlock << " - "<< wallRes.score << std::endl;
    if (idPlayerToBlock >= 0)
    {
        if (wallRes.score > 0.0)
        {
            std::ostringstream Output;
            Output << wallRes.x << " " << wallRes.y << " " << wallRes.orientation;
            if (! wallRes.text.empty())
            {
                Output << " " << wallRes.text;
            }
            ret = Output.str();

            aPlayers[idPlayerToBlock].IncNbTimesBlocked();
        }
    }

    return ret;
}

/**
 * @brief   Evaluation of wall placement
 * @param[in]     aPlayers    Player list
 * @param[in,out] aBoard      Game board
 * @param[in]     aScoreBoard Scoreboard - current evaluation of all players
 */
std::string WallPlacer::PlaceWall (PlayerList& aPlayers, Board& aBoard, const ScoreBoard& aScoreBoard)
{
    std::string     ret;
    const Player&   me = aPlayers[aPlayers.GetMyId()];

    // Wall selection is made on score
    mBestChoice = eBestScore;

    // Preliminary checks
    if (    (me.GetWallsLeft() > 0)
            &&  Filter3P_Basic(aPlayers) )
    {
        const PlayerList::TIdVector&          othersId = aPlayers.GetOthersIds();
        PlayerList::TIdVector::const_iterator iId;
        WallResult      wallPlacement;

        // Iterate through others players ids
        for (iId = othersId.begin(); iId != othersId.end(); ++iId)
        {
            Player& badGuy = aPlayers[*iId];

            if (badGuy.IsAlive())
            {
                // Find best result
                WallResult  wallResult = PlaceWallPath (aPlayers, *iId, aBoard, aScoreBoard);
                const CMove& badGuyMove = aScoreBoard[*iId];
                const CMove& myMove = aScoreBoard[aPlayers.GetMyId()];

                if (wallResult.distance < std::numeric_limits<float>::infinity())
                {
                    bool bPlaceWall = false;

                    // Always place a wall if bad guy is near its aim
                    bPlaceWall |= (badGuyMove.currentDistance <= 1.0);
                    if (aPlayers.CountAlive() < 3)
                    {
                        bPlaceWall |= (badGuyMove.currentDistance <= 2.0);
                    }

                    if (badGuy.GetNbTimesBlocked() > 0)
                    {
                        // We should not place a wall if we might be handicaped by it
                        // Compare the distance of the other play   er on my distance matrix to my distance.
                        float otherDistanceFromMyView = aBoard.GetCell (badGuy.GetPosition()).mPlayerDistances[aPlayers.GetMyId()];
                        bPlaceWall = (otherDistanceFromMyView > myMove.currentDistance);
                        
                        bPlaceWall |= (wallResult.increase >= 5.0);

                        // Place a wall if the distance increase for bad guy is twice our
                        bPlaceWall |= (wallResult.increase > 2.0) && (wallResult.increase > (wallResult.myIncrease*2));
                    }

                    // Do not place wall is it doesn't increase the bad guy distance significantly
                    bPlaceWall &= (wallResult.increase > (badGuyMove.currentDistance / 10));

                    if (bPlaceWall)
                    {
                        wallPlacement = wallResult;
                        badGuy.IncNbTimesBlocked();
                        break;
                    }
                }
            }
        }

        // If a wall placement was decided ...
        if (wallPlacement.score > 0.0)
        {
            // .. convert it to a text form
            std::ostringstream Output;
            Output << wallPlacement.x << " " << wallPlacement.y << " " << wallPlacement.orientation;
            if (! wallPlacement.text.empty())
            {
                Output << " " << wallPlacement.text;
            }
            ret = Output.str();
        }
    }

    return ret;
}

/**
 * @brief   Keep the best result
 * @param[in,out] aResult     Current result
 * @param[in]     aChallenger Challenger to compare to
 */
void WallPlacer::Best (WallResult& aResult, const WallResult& aChallenger)
{
    bool    bBetter;

    switch (mBestChoice)
    {
    case eBestScore :
        bBetter = (aChallenger.score > aResult.score);
        break;
    case eBestDistance :
        bBetter = (aChallenger.distance > aResult.distance);
        break;
    case eBestIncrease :
        bBetter = (aChallenger.increase > aResult.increase);
        break;
    default:
        bBetter = false;
        break;
    }

    if (bBetter)
    {
        aResult = aChallenger;
    }
}

/**
 * @brief   Simple wall placement algorithm
 *
 * Only examine walls surrounding the player
 *
 * @param[in] aPlayers          Players list
 * @param[in] aPlayerIdToBlock  Which player should we block ?
 * @param[in] aBoard            Game board (modified in place for computations)
 * @param[in] aScoreBoard       Score board before the wall placement
 *
 * @return  A best choice for blocking the selected player
*/
WallResult WallPlacer::PlaceWallSimple (const PlayerList& aPlayers, int aPlayerIdToBlock, Board& aBoard, const ScoreBoard& aScoreBoard)
{
    WallResult  Result;

    if (aPlayerIdToBlock >= 0)
    {
        int x = aPlayers[aPlayerIdToBlock].GetPosition().x;
        int y = aPlayers[aPlayerIdToBlock].GetPosition().y;
        Result =      TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y,   'H');
        Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x-1, y,   'H'));
        Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y,   'V'));
        Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y-1, 'V'));
        Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x+1, y,   'V'));
        Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x+1, y-1, 'V'));
        Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y+1, 'H'));
        Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x-1, y+1, 'H'));
    }

    return Result;
}

/**
 * @brief   wall placement on the shortest path of the opponent
 *
 * Try every wall on the shortest path of the opponent
 *
 * @param[in] aPlayers          Players list
 * @param[in] aPlayerIdToBlock  Which opponent to consider
 * @param[in] aBoard            Game board (modified in place for computations)
 * @param[in] aScoreBoard       Score board before the wall placement
 *
 * @return  A best choice for blocking the selected player
*/
WallResult WallPlacer::PlaceWallPath (const PlayerList& aPlayers, int aPlayerIdToBlock, Board& aBoard, const ScoreBoard& aScoreBoard)
{
    WallResult  Result;

    if (aPlayerIdToBlock >= 0)
    {
        PlayerDistanceMatrix    playerDistance(aBoard, aPlayerIdToBlock);
        BoardWeights            boardWeight (aBoard, playerDistance);
        Path::const_iterator    iPath;
        Path                    path = boardWeight.ShortestPath(aPlayers[aPlayerIdToBlock].GetPosition());
        for (iPath=path.begin(); iPath!=path.end(); ++iPath)
        {
            int x = iPath->x;
            int y = iPath->y;

            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y,   'H'));
            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x-1, y,   'H'));
            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y,   'V'));
            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y-1, 'V'));
            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x+1, y,   'V'));
            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x+1, y-1, 'V'));
            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x,   y+1, 'H'));
            Best (Result, TestWall (aPlayers, aPlayerIdToBlock, aScoreBoard, aBoard, x-1, y+1, 'H'));
        }
    }

    return Result;
}


/**
 * @brief   Evaluate a wall placement
 *
 * Try every wall on the shortest path of the opponent
 *
 * @param[in] aPlayers      Players list
 * @param[in] aPlayerId     Which opponent to consider
 * @param[in] aBoard        Game board (modified in place for computations)
 * @param[in] aScoreBoard   Score board before the wall placement
 * @param[in] aX            Wall position, x coordinate
 * @param[in] aY            Wall position, y coordinate
 * @param[in] aOrientation  Wall position, orientation
 *
 * @return  An evaluation of what would be the situation after placing the wall
*/
WallResult WallPlacer::TestWall (const PlayerList& aPlayers, int aPlayerId, const ScoreBoard& aScoreBoard, Board& aBoard, int aX, int aY, char aOrientation) const
{
    WallResult      result(aX, aY, aOrientation);
    const Player&   player  = aPlayers[aPlayerId];
    ScoreBoard      scoreBoard;

    // Verify new wall won't collide existing ones
    if (! aBoard.Collide (aX, aY, aOrientation))
    {
        // Place the wall an evaluate new distances
        aBoard.AddWall (aX, aY, aOrientation);
        scoreBoard.Build (aPlayers, aBoard);

        // discard if one player is blocked by the new wall
        if (! scoreBoard.OnePlayerBlocked())
        {
            // compute a score
            result.distance = scoreBoard[aPlayerId].moveDistance;
            result.flightDistance = FlightDistance(player, aBoard);
            result.increase = scoreBoard[aPlayerId].currentDistance - aScoreBoard[aPlayerId].currentDistance;
            result.myIncrease = scoreBoard[aPlayers.GetMyId()].currentDistance - aScoreBoard[aPlayers.GetMyId()].currentDistance;
            result.score = result.distance;
            result.score += result.increase * 2;
            result.score -= result.myIncrease * 3;

            // AKA !
            if (result.increase > 7)
            {
                result.text = "Broke your legs";
            }
            else if (result.increase > 5)
            {
                result.text = "Take a side road";
            }
            else if (result.increase > 3)
            {
                result.text = "Chuck's move";
            }
        }
        aBoard.DelWall (aX, aY, aOrientation);
    }

    return result;
}


/**
 * @brief   Game object
 *
 * Receive inputs and gives an instruction in return
*/
class Game
{
public:
    /// Shared pointer on a game
    typedef shared_ptr<Game> Ptr;

    Game (int aWidth, int aHeight, int aPlayerCount, int aMyId);
    void        UpdatePlayer (int aId, int aX, int aY, int aWallsLeft);
    void        AddWall      (int aX, int aY, char aOrientation);
    std::string MyTurn (void);

private:
    PlayerList  mPlayers;   //!< List of players
    Board       mBoard;     //!< Game board
};


/**
 * @brief   Constructor
*/
Game::Game (int aWidth, int aHeight, int aPlayerCount, int aMyId) :
    mPlayers    (aPlayerCount, aMyId),
    mBoard      (aWidth, aHeight)
{
}

/**
 * @brief   Update a player
 *
 * @param[in]   aId         Player ID to update
 * @param[in]   aX          X position of the player
 * @param[in]   aY          Y position of the player
 * @param[in]   aWallsLeft  Numbers of walls left
*/
void Game::UpdatePlayer (int aId, int aX, int aY, int aWallsLeft)
{
    mPlayers.UpdatePlayer (aId, aX, aY, aWallsLeft);
}

/**
 * @brief   Add a wall
 *
 * @param[in]   aX          X position of the wall
 * @param[in]   aY          Y position of the wall
 * @param[in]   aOrientation  direction of the wall ('H' or 'V')
*/
void Game::AddWall (int aX, int aY, char aOrientation)
{
    mBoard    .AddWall (aX, aY, aOrientation);
}


/**
 * brief    Play a turn
*/
std::string  Game::MyTurn (void)
{
    // Begin time measurement
    CTimeSingleton::GetInstance().Start();

    std::string     cmd;        // Resulting command
    ScoreBoard      scoreBoard; // Score board for all players
    WallPlacer      wallPlacer; // Wall placing algorithm

    // Build score board
    scoreBoard.Build (mPlayers, mBoard);
    scoreBoard.PrintDebug (mPlayers);

    // Evaluate placing a wall
    cmd = wallPlacer.PlaceWall(mPlayers, mBoard, scoreBoard);
    if (cmd.empty())
    {
        // No wall placement, do a move
        cmd = ToString(scoreBoard[mPlayers.GetMyId()].moveDirection);
    }

    // Trace time consumed for this turn
    std::cerr << "Turn took " << std::setprecision(5) << CTimeSingleton::GetInstance().Elapsed() << "ms" << std::endl;

    return cmd;
}


/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
int main()
{
    int w; // width of the board
    int h; // height of the board
    int playerCount; // number of players (2 or 3)
    int myId; // id of my player (0 = 1st player, 1 = 2nd player, ...)
    cin >> w >> h >> playerCount >> myId;
    cin.ignore();
    Game::Ptr   GamePtr (new Game(w, h, playerCount, myId));

    // game loop
    while (1)
    {
        for (int i = 0; i < playerCount; i++)
        {
            int x; // x-coordinate of the player
            int y; // y-coordinate of the player
            int wallsLeft; // number of walls available for the player
            cin >> x >> y >> wallsLeft;
            cin.ignore();
            GamePtr->UpdatePlayer(i, x, y, wallsLeft);
        }
        int wallCount; // number of walls on the board
        cin >> wallCount;
        cin.ignore();
        for (int i = 0; i < wallCount; i++)
        {
            int wallX; // x-coordinate of the wall
            int wallY; // y-coordinate of the wall
            string wallOrientation; // wall orientation ('H' or 'V')
            cin >> wallX >> wallY >> wallOrientation;
            cin.ignore();
            GamePtr->AddWall(wallX, wallY, wallOrientation[0]);
        }

        cout << GamePtr->MyTurn() << endl;
    }
}
