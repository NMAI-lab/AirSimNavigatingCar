/**
 * Definition of the map
 * The map is the shape of the letter 'T'.
 */

locationName(post1,[47.6414823712,-122.140364991]).
locationName(post2,[47.6426242556,-122.140354517]).
locationName(post3,[47.642632115856806,-122.13892325834075]).
locationName(post4,[47.642634517703016,-122.14203898419318]).
 
// Possible map transitions.
// possible(StartingPosition, PossibleNewPosition)
possible(post1,post2).
possible(post2,post3).
possible(post2,post4).

