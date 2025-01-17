(define (domain robot_domain)
  (:requirements :strips :typing :durative-actions)

  ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:types
    robot
    waypoint
  );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:predicates
    (robot_at ?r - robot ?wp - waypoint)
    (inspected ?wp - waypoint)
  );; end Predicates ;;;;;;;;;;;;;;;;;;;;

  ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration (= ?duration 5)
    :condition (and
        (at start (robot_at ?r ?wp1))
    )
    :effect (and
        (at start (not (robot_at ?r ?wp1)))
        (at end (robot_at ?r ?wp2))
    )
)

  (:durative-action inspect
    :parameters (?r - robot ?wp - waypoint)
    :duration (= ?duration 5)
    :condition (and
        (at start (robot_at ?r ?wp))
    )
    :effect (and
        (at end (inspected ?wp))
    )
)
);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;

