/**
 * Definition of the map
 * The map is the shape of the letter 'T'.
 */
 
 /**
 * Definition of the map
 * The map is the shape of the letter 'T'.
 * 
 * 4----2---5----3
 * 		|
 * 		|
 * 		|
 * 		1
 */

locationName(post1,[47.6414823712,-122.140364991]).
locationName(post2,[47.6426242556,-122.140354517]).
locationName(post3,[47.642632115856806,-122.13892325834075]).
locationName(post4,[47.642634517703016,-122.14203898419318]).

// in the shadow
//locationName(post5,[47.6426152931,-122.139873466]).

// next to the car
locationName(post5,[47.6426234412,-122.139749573]).

// Possible map transitions.
// possible(StartingPosition, PossibleNewPosition)
possible(post1,post2).
possible(post2,post5).
possible(post2,post4).
possible(post5,post3).




