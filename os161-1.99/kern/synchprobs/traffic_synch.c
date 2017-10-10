#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */


//previous prototypes
bool right_turn(Direction origin, Direction destination);
bool canEnter(Direction origin, Direction destination);

typedef struct Vehicle {
  Direction origin;
  Direction destination;
} Vehicle;


/*  vehicles in the intersection */
/*  note: this is an array of volatile pointers */
//bigger than we will probably need
struct Vehicle * volatile inIntersection[100];
int inIndex=0;


/* lock to protect vehicles array 
   i.e only one vehicle can enter the intersection at a time
*/
static struct lock *intersectionLock;

static struct cv *intersectionCV;

/*
 * bool right_turn()
 * 
 * Purpose:
 *   predicate that checks whether a vehicle is making a right turn
 *
 * Arguments:
 *   a pointer to a Vehicle
 *
 * Returns:
 *   true if the vehicle is making a right turn, else false
 *
 * Note: written this way to avoid a dependency on the specific
 *  assignment of numeric values to Directions
 */
bool
right_turn(Direction origin, Direction destination) {
  if (((origin == west) && (destination == south)) ||
      ((origin == south) && (destination == east)) ||
      ((origin == east) && (destination == north)) ||
      ((origin == north) && (destination == west))) {
    return true;
  } else {
    return false;
  }
}

//check if the 'new car' violated the conditions with any cars already in the intersection
bool canEnter(Direction origin, Direction destination) {
  /* compare newly-added vehicle to each other vehicles in the intersection */
  for(int i=0;i<inIndex;i++) {
    /* conflict if both vehicles do NOT have the same origin */
    if (inIntersection[i]->origin == origin) return false;
    /* conflict if vehicles do NOT go in opposite directions */
    if ((inIntersection[i]->origin != destination) ||
        (inIntersection[i]->destination != origin)) return false;
    /* conflict if NOT one makes a right turn and 
       the other has a different destination */
    if (!((right_turn(inIntersection[i]->origin,inIntersection[i]->destination) || right_turn(origin,destination)) &&
        (destination != inIntersection[i]->destination))) return false;;
  }	
  //tried real hard but didnt find a conflict, go ahead
  return true;
}

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  intersectionLock = lock_create("intersectionLock");
  if (intersectionLock == NULL) {
    panic("could not create intersection lock");
  }
  intersectionCV = cv_create("intersectionCV");
  if (intersectionCV == NULL) {
    panic("could not create intersection CV");
  }
  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  KASSERT(intersectionLock != NULL);
  lock_destroy(intersectionLock);
  KASSERT(intersectionCV != NULL);
  cv_destroy(intersectionCV);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{

  Vehicle *newCar = kmalloc(sizeof(struct Vehicle));
  newCar->origin = origin;
  newCar->destination = destination;

  //HEY i want to enter intersection gimmie the lock
  lock_acquire(intersectionLock);

  //check if this car violated condition with any currently inIntersection
  //we wrap it in a while loop so it re-checks when we wake back up
  while(!canEnter(origin,destination)) {
        //if it cant enter the intersection wait until something changes
	cv_wait(intersectionCV,intersectionLock);
  }
  
  //we can go into the intersection now!
  inIntersection[inIndex] = newCar;
  ++inIndex;

  //dont need the lock anymore
  lock_release(intersectionLock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  //get lock to make sure people arnt trying to enter while we are exiting
  lock_acquire(intersectionLock);

  //find car thats leaving and remove it from intersection
  for(int i=0;i<inIndex;++i) {
	if(inIntersection[i]->origin == origin && inIntersection[i]->destination == destination) {
		//NOTE: this potentailly could be an issue if we started identifying cars by more than their path
		--inIndex;
		inIntersection[i] = inIntersection[inIndex];
		//car has left so we tell the waiting cars to try again
		cv_broadcast(intersectionCV,intersectionLock);
		break;
	}
  }
  //dont need the lock anymore
  lock_release(intersectionLock);

}
