(define (domain hospital_nav)

;remove requirements that are not needed
(:requirements :strips :typing :conditional-effects :equality :negative-preconditions :universal-preconditions :durative-actions)

(:types
  door
  zone
  room
  robot
)

(:predicates ;todo: define predicates here
  (robot_at_zone ?r - robot ?z - zone)
  (robot_at_room ?r - robot ?ro - room)
  (closed_door ?d - door)
  (opened_door ?d - door)
  (object_at ?r - room)


  (connected_by_door ?r1 - room ?d - door)
  (connected ?r1 ?r2 - room)
  (zone_at_room ?z - zone ?r - room)
  (object_at_zone ?z - zone)
  (object_at_robot ?r - robot)
)

(:durative-action move_room2room
  :parameters (?r - robot ?from ?to - room)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_room ?r ?from))
    (at start(connected ?from ?to))    
  )
  :effect (and
    (at end(robot_at_room ?r ?to))
    (at end(not (robot_at_room ?r ?from)))
  )
)

;; Cross door
(:durative-action move_room2room_through_door
  :parameters (?r - robot ?from ?to - room ?d - door)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_room ?r ?from))
    (at start(connected_by_door ?from ?d))
    (at start(connected_by_door ?to ?d))
    (at start(not (= ?from ?to)))
    (at start(opened_door ?d))
  )
  :effect (and
    (at end(robot_at_room ?r ?to))
    (at end(not (robot_at_room ?r ?from)))
  )
)

(:durative-action open_door
  :parameters (?r - robot ?r1 - room ?d - door)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_room ?r ?r1))
    (at start(connected_by_door ?r1 ?d))
    (at start(closed_door ?d))
  )
  :effect (and 
    (at end(opened_door ?d))
    (at end(not (closed_door ?d)))
  )
)

(:durative-action close_door
  :parameters (?r - robot ?r1 - room ?d - door)
  :duration (= ?duration 5)
  :condition (and 
    (at start(robot_at_room ?r ?r1))
    (at start(connected_by_door ?r1 ?d))
    (at start(opened_door ?d))
  )   
  :effect (and 
    (at end (closed_door ?d))
    (at end(not (opened_door ?d)))
  )
)

(:durative-action entry_zone
  :parameters (?r - robot ?z - zone ?ro - room)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_room ?r ?ro))
    (at start (zone_at_room ?z ?ro))
  )
  :effect (and
    (at end(robot_at_zone ?r ?z))
    (at end(not (robot_at_room ?r ?ro)))
  )
)

(:durative-action exit_zone
  :parameters (?r - robot ?z - zone ?ro - room)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_zone ?r ?z))
    (at start (zone_at_room ?z ?ro))
  )
  :effect (and
    (at end(not (robot_at_zone ?r ?z)))
    (at end(robot_at_room ?r ?ro))
  )
)

(:durative-action take_object_from_room
  :parameters (?r - robot ?ro - room)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_room ?r ?ro))
    (at start(object_at ?ro))
  )
  :effect (and
    (at end(object_at_robot ?r))
    (at end(not (object_at ?ro)))
  )
)
 
(:durative-action take_object_from_zone
  :parameters (?r - robot ?z - zone ?ro - room)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_zone ?r ?z))
    (at start(zone_at_room ?z ?ro))
    (at start(object_at_zone ?z))
  )
  :effect (and 
    (at end(object_at_robot ?r))
    (at end(not (object_at_zone ?z)))
  )
)

(:durative-action drop_object_in_zone
  :parameters (?r - robot ?z - zone ?ro - room)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_zone ?r ?z))
    (at start(zone_at_room ?z ?ro))
    (at start(object_at_robot ?r))
  )
  :effect (and
    (at end(object_at_zone ?z))
    (at end(not (object_at_robot ?r)))
  )
)

(:durative-action drop_object_in_room
  :parameters (?r - robot ?ro - room)
  :duration (= ?duration 5)
  :condition (and
    (at start(robot_at_room ?r ?ro))
    (at start(object_at_robot ?r))
  )
  :effect (and 
    (at end(object_at ?ro))
    (at end(not (object_at_robot ?r)))
  )
)

)