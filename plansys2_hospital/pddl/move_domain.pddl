;Header and description

(define (domain move_domain)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :typing)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
  room robot - stuff_locs
  stuff
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
  (connected ?r1 ?r2 - room)
  (robot_at ?r - robot ?site - room)
  (stuff_at ?st - stuff ?site - stuff_locs)
  (gripper_free ?r - robot)
)


(:functions ;todo: define numeric functions here
)

;define actions here
(:durative-action move
  :parameters (?rob - robot ?from ?to - room)
  :duration (= ?duration 5)
  :condition (and 
    (over all(connected ?from ?to))
    (over all(connected ?to ?from))
    (at start(robot_at ?rob ?from))
  )
  :effect (and 
    (at start(robot_at ?rob ?to))
    (at start(not (robot_at ?rob ?from)))
  )
)

(:durative-action take_stuff
  :parameters (?rob - robot ?from - room ?st - stuff)
  :duration (= ?duration 3)
  :condition (and 
    (at start(gripper_free ?rob))
    (at start(stuff_at ?st ?from))
    (at start (robot_at ?rob ?from))
  )
  :effect (and 
      (at end(not (gripper_free ?rob)))
      (at start(stuff_at ?st ?rob))
      (at start(not (stuff_at ?st ?from)))
    )
)

(:durative-action release_stuff
  :parameters (?rob - robot ?to - room ?st - stuff)
  :duration (= ?duration 3)
  :condition (and 
    (at start (robot_at ?rob ?to))
    (at start (stuff_at ?st ?rob))  
  )
  :effect (and 
    (at end (gripper_free ?rob))
    (at start (stuff_at ?st ?to))
    (at end (not (stuff_at ?st ?rob)))
  )
)



)