(declare-const a Real)
(declare-const b Real)
(declare-const c Real)
(assert (>= a 0))
(assert (>= b 0))
(assert (>= c 0))
(assert (and (<= a -1) (<= a 0) (<= a 1)))
(check-sat)
