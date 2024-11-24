#include "boids.hpp"

/* OPERAZIONI TRA ARRAY */

couple operator+(couple const &add1, couple const &add2)
{
    double a = add1[0] + add2[0];
    double b = add1[1] + add2[1];
    return couple{a, b};
} // somma di due array

couple operator-(couple const &add1, couple const &add2)
{
    double a = add1[0] - add2[0];
    double b = add1[1] - add2[1];
    return couple{a, b};
} // differenza tra due array

couple operator*(double scal, couple const &vet)
{
    return couple{scal * vet[0], scal * vet[1]};
} // prodotto tra coppia e scalare

double squaresum(couple i)
{
    return (i[0] * i[0] + i[1] * i[1]);
} // quadrato del modulo (per una coppia "i" di dati in generale)

/* METODI DELLA CLASSE BOID */
// bds è l'array
// boid è la classe e anche il costruttore

bds::boid::boid(couple p, couple s) : pos_{p}, vel_{s} {} // costruttore di base, forse da togliere
bds::boid::boid() : pos_{0., 0.}, vel_{0., 0.} {}         // costruttore senza parametri, adesso restituisce boid fermi all'origine ma va implementato con la funzione random

void bds::boid::vel_mod(couple s)
{
    vel_ = s + vel_; // si dovrebbe implementare l'operatore +=, credo che a giacomini piaccia
} // modifica la velocità dato in input un array di "modifica" chiamato "s"; vel_ è un membro privato della classe boid.

void bds::boid::pos_mod(double deltat)
{
    pos_ = pos_ + (deltat * vel_);
} // modifica la posizione, i parametri di velocità sono presi dal corpo della funzione; la posizione non può essere (per come è messa ora) modificata a piacere
// posizione viene modificata aggiungendo la velocità moltiplicata per delta t (tempo)
// assert sono da inserire nel ciclo for e nel costruttore

couple bds::boid::pos() const { return pos_; }
couple bds::boid::vel() const { return vel_; } // funzioni che uso per cavare fuori velocità e posizione dal boid

/* FUNZIONI LIBERE */

double bds::squaredistance(bds::boid i) // in pratica prende in imput un boid i
{
    couple p = i.pos();  // di questo boid cava fuori le due coordinate della posizione in un array chiamato "p",
    return squaresum(p); // poi fa il quadrato del modulo del vettore posizione
}
// non sono veramente sicura serva

bool bds::BoidsAreNear(bds::boid i, bds::boid j, double dist)
{
    if (i.pos() == j.pos() && i.vel() == j.vel())
    {
        return 0;
    } // eventualmente definisci uguaglianza tra boids
    couple diff = (i.pos() - j.pos()); // array "vettore posizione differenza" tra i vettori pos di due boids.
    double diffdist = squaresum(diff); // quadrato del modulo del vett. pos. differenza
    return diffdist < (dist * dist);   // il quadrato del modulo è minore di una certa distanza al quadrato???
} // controllo vicini, dato che prende in input una distanza di "vicinanza" la uso anche per la distanza di separazione
// in input anche due generici boid i e boid j

std::vector<bds::boid> bds::create_boids_vector(lu_int n)
{ // ci sono i lu_int perchè alla compilazione mi dava problemi di conversione (perchè la dimensione del vettore è long unsigned)
    std::vector<bds::boid> boid_v;
    for (lu_int i = 0; i <= n - 1; ++i)
    {
        bds::boid a{};       // prende un boid "a"
        boid_v.push_back(a); // lo aggiunge alla fine del vettore con il metodo pushback
    }
    assert(boid_v.size() == n);
    return boid_v;
} // funzione che, prendendo in input un n, crea l'insieme di boids sotto forma di vettore chiamato boid_v (il tipo del vettore è infatti bds::boid)
// aggiunta di un assert che verifica che il numero di boids (dimensione del vector) rimanga n

// std::vector<bds::boid> bds::inizialization(lu_int n_boids)
// {
//     std::vector<bds::boid> boid_vector = bds::create_boids_vector(n_boids); // inizializza il boid vector tramite la funzione di prima dato in input n boids
//   return boid_vector;
// } // funzione che inizializza il boid in funzione del numero inserito: va chiamata nel main perchè prima n_boids non ha valore

couple bds::v_separation(lu_int i, double sep_dist, double sep_fact, std::vector<boid> boid_vector)
{
    couple v_sep{0., 0.};
    assert(i < boid_vector.size()); // si mette l'assert perchè tanto per come sarà formulato il ciclo, questa casistica non deve avvenire

    boid i_boid = boid_vector[i]; // i-esimo boid
    for (lu_int j = 0; j < boid_vector.size(); ++j)
    {
        if (j != i)
        {
            boid j_boid = boid_vector[j];               // j-esimo boid
            if (BoidsAreNear(i_boid, j_boid, sep_dist)) // sep-dist da settare?
            {                                           // se il boid j è vicino al boid i ... applico il controllo dei vicini con la distanza di separazione
                v_sep = v_sep + (j_boid.pos() - i_boid.pos());
            }
        }

        v_sep = (-1 * sep_fact) * v_sep; // fattore di separazione negativo??
    }
    return v_sep;
} // regola di separazione
// ==> restituisce un array di coordinate di velocità

couple bds::v_alignment(lu_int i, double alig_fact, std::vector<boid> boid_vector)
{
    couple v_alig{0., 0.};
    assert(i < boid_vector.size());

    boid i_boid = boid_vector[i];

    for (lu_int j = 0; j < boid_vector.size(); ++j)
    { // nessun controllo vicini perchè la consegna non lo richiede
        if (j != i)
        {
            boid j_boid = boid_vector[j];
            v_alig = v_alig + j_boid.vel();
        }

        v_alig = (1. / static_cast<double>(boid_vector.size() - 1) * v_alig);
        v_alig = v_alig - i_boid.vel();
    }

    v_alig = alig_fact * v_alig; // conversione da gestire con static_cast
    return v_alig;
} // regola di allineamento ==> restituisce un array di coordinate di velocità

couple bds::v_coesion(lu_int i, double dist_vic, double coes_fact, std::vector<boid> boid_vector)
{
    /* sarebbe carino trovare il modo di usare l'algoritmo std::accumulate, ma non so se si riesce */
    couple c_mass;
    assert(i < boid_vector.size());
    int near_boids{1};
    boid i_boid = boid_vector[i];
    for (lu_int j = 0; j < boid_vector.size(); ++j)
    {
        boid j_boid = boid_vector[j];
        if (BoidsAreNear(i_boid, j_boid, dist_vic))
        {
            c_mass = c_mass + j_boid.pos();
            ++near_boids;
        }
    }
    c_mass = (1. / static_cast<double>(near_boids - 1)) * c_mass; // come prima static cast
    // ho sommato tutti i boid vicini ma quello iniziale è escluso: ho la posizione del centro di massa - la posizione del primo

    couple v_coes = coes_fact * c_mass;
    return v_coes;
} // regola di coesione ==> restituisce un array di coordinate di velocità

void bds::v_mod(lu_int i, double sep_fact, double sep_dist, double alig_fact, double dist_vic, double coes_fact, std::vector<boid> &boid_vector)
{
    couple v_mod = v_separation(i, sep_dist, sep_fact, boid_vector) + v_alignment(i, alig_fact, boid_vector) + v_coesion(i, dist_vic, coes_fact, boid_vector);
    boid_vector[i].vel_mod(v_mod); // la velocità del boid viene modificata dalla funzione vel_mod a cui viene dato in input l'array v_mod precedentemente "creato"
} // funzione che applica le modifiche di velocità legate alle regole al boid stesso, va iterata nel ciclo for per ogni boid

void bds::p_mod(lu_int i, std::vector<boid> boid_vector, double deltat)
{
    boid_vector[i].pos_mod(deltat);
} // funzione che forse non è necessaria, può essere implementata direttamente nel ciclo for; modifica la posizione in base alle velocità

/*FUNZIONI CHE ESTRAGGONO VALORI STATISTICI*/

double bds::GetMeanDistance(std::vector<boid> boid_vector, double sep_dist)
{
    double dist_sum{0.};
    int n_events{0};
    for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        for (lu_int j = 0; j < boid_vector.size(); ++j)
        {
            boid j_boid = boid_vector[j];
            if (BoidsAreNear(i_boid, j_boid, sep_dist))
            { // ho messo un controllo vicini qua perchè non credo abbia senso considerare nel calcolo della media la distanza tra i boid megalontani (ma forse lo ha...)
                dist_sum += sqrt(squaresum(i_boid.pos() - j_boid.pos()));
                ++n_events; // perchè non so a priori quante sono le coppie di boids che contribuiscono alla determinazione della distanza media
            }
        }
    }
    double meandist = dist_sum / n_events;
    return meandist;
} // tira fuori la distanza media tra i boids di un dato vettore, da chiamare nel ciclo

double bds::GetMeanVelocity(std::vector<boid> boid_vector)
{
    double vel_sum{0.};
    int n_events{0};
    for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        vel_sum += sqrt(squaresum(i_boid.vel()));
        ++n_events;
    }
    double meanvel = vel_sum / n_events;
    return meanvel;
} // tira fuori la velocità media dei boids, da chiamare nel ciclo

double bds::GetStdDevDistance(std::vector<boid> boid_vector, double sep_dist)
{
    double meandist = GetMeanDistance(boid_vector, sep_dist);
    double square_dist_sum{0.};
    int n_events{0};
    for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        for (lu_int j = 0; j < boid_vector.size(); ++j)
        {
            boid j_boid = boid_vector[j];
            if (BoidsAreNear(i_boid, j_boid, sep_dist))
            {
                square_dist_sum += squaresum(i_boid.pos() - j_boid.pos());
                ++n_events;
            }
        }
    }
    double stddev = (square_dist_sum / n_events) - (meandist * meandist);
    return stddev;
} // calcolo della deviazione standard come vuole la sara magica (quadrato della media - media dei quadrati)

double bds::GetStdDevVelocity(std::vector<boid> boid_vector)
{
    double square_vel_sum{0.};
    int n_events{0};
    for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        square_vel_sum += squaresum(i_boid.vel());
        ++n_events; // inutile ma tatino
    }
    double meanvel = GetMeanVelocity(boid_vector);
    double stddev = (square_vel_sum / n_events) - (meanvel * meanvel);
    return stddev;
} // come prima

void Pacman(std::vector<bds::boid> &boid_vector, double field_width, double field_height)
{
    field_width = 15.;
    field_height = 10.;
    for (lu_int i{0}; i < boid_vector.size(); i++)
    {
        if ((boid_vector[i].pos())[0] < -(field_width / 2))
        {
            (boid_vector[i].pos())[0] += field_width;
        }
        if ((boid_vector[i].pos())[0] > (field_width / 2))
        {
            (boid_vector[i].pos())[0] -= field_width;
        }
        if ((boid_vector[i].pos())[1] < -(field_height / 2))
        {
            (boid_vector[i].pos())[1] += field_height;
        }
        if ((boid_vector[i].pos())[1] > (field_height / 2))
        {
            (boid_vector[i].pos())[1] -= field_height;
        }
    }
}



/*il programma compila senza warning ma ancora non so se funzioni
bisogna: creare il main in un file a parte, gestire il controllo della velocità e l'effetto pacman per il bordo campo,
fare il file doctest per i test, inserire libreria grafica o qualunque miglioramento, inizializzare con un estrattore di numeri casuali i boid
*/

/* bds::boid i_boid = boid_vector[i];
       boid_vector[i].pos() =
           i_boid.pos() = {dist(eng), dist(eng)};*/