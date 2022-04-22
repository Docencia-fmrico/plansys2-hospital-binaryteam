;Header and description

(define (domain move_domain)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :typing)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
  room
  robot
  zone
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
  (connected ?r1 ?r2 - room)
  (robot_at ?r - robot ?site - room)
)


(:functions ;todo: define numeric functions here
)

;define actions here
(:durative-action move
  :parameters (?rob - robot ?from ?to - room)
  :duration (= ?duration 5)
  :condition (and 
    (at start (and 
      (connected ?from ?to)
      (connected ?to ?from)
      (robot_at ?rob ?from)
      (robot_at_zone ?zon)
    ))
  )
  :effect (and 
    (at end (and 
      (robot_at ?rob ?to)
      (not (robot_at ?rob ?from))
    ))
  )
)


)