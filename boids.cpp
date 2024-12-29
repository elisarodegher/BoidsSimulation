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
    auto a = i.begin();
    auto b = i.end();
    double sum = std::inner_product(a, b, a, 0);
    return sum; //da testare!!
} // quadrato del modulo (per una coppia "i" di dati in generale)

/* METODI DELLA CLASSE BOID */
// bds è l'array
// boid è la classe e anche il costruttore

bds::boid::boid(couple p, couple s) : pos_{p}, vel_{s} {} // costruttore di base, forse da togliere

void bds::boid::vel_mod(couple s)
{
    vel_ = vel_ + s; // si dovrebbe implementare l'operatore +=, credo che a giacomini piaccia
} // modifica la velocità dato in input un array di "modifica" chiamato "s"; vel_ è un membro privato della classe boid.

void bds::boid::pos_mod(double deltat)
{
    pos_ = pos_ + (deltat * vel_);
} // modifica la posizione, i parametri di velocità sono presi dal corpo della funzione; la posizione non può essere (per come è messa ora) modificata a piacere
// posizione viene modificata aggiungendo la velocità moltiplicata per delta t (tempo)
// assert sono da inserire nel ciclo for e nel costruttore
void bds::boid::pos_mod(couple p)
{
    pos_ = pos_ + p;
}

couple bds::boid::pos() const { return pos_; }
couple bds::boid::vel() const { return vel_; } // funzioni che uso per cavare fuori velocità e posizione dal boid
double bds::boid::get_angle() const
{
    double angle = atan2(vel_[1], vel_[0]);
    return angle;
}
bool bds::operator==(boid i, boid j) {
    return (i.pos() == j.pos() && i.vel() == j.vel());
}
bool bds::operator!=(boid i, boid j) {
    return (i.pos() != j.pos() || i.vel() != j.vel());
}

bds::fperiod::fperiod(double per) : per_{fabs(per)} {}
auto bds::fperiod::operator() (double num) {
    if (num > (per_ / 2)) {
        num -= per_;
    }
    if (num < (-per_ / 2)) {
        num += per_;
    }
}



/* FUNZIONI LIBERE */
void bds::periodize(double &num, double per) {
    if (num > (fabs(per) / 2)) {
        num -= per;
    }
    if (num < (-fabs(per) / 2)) {
        num += per;
    }
}

bool bds::BoidsAreNear(bds::boid i, bds::boid j, double dist, double field_width, double field_height)
{
    if (i == j)
    {
        return 0;
    }

    couple diff = (i.pos() - j.pos()); 
    periodize(diff[0], field_width);
    periodize(diff[1], field_height); // array "vettore posizione differenza" tra i vettori pos di due boids.

    double diffdist = squaresum(diff);
    return diffdist < (dist * dist);
} // controllo vicini, dato che prende in input una distanza di "vicinanza" la uso anche per la distanza di separazione

couple bds::v_separation(lu_int i, double sep_dist, double sep_fact, std::vector<boid> boid_vector, double field_width, double field_height)
{
    /*std::vector<couple> pos_vec;
    auto makepos = [&pos_vec] (boid& i_boid) { 
        couple p = i_boid.pos();
        pos_vec.push_back(p);
    };
    
    assert(i < boid_vector.size()); // si mette l'assert perchè tanto per come sarà formulato il ciclo, questa casistica non deve avvenire
    
    std::for_each(boid_vector.begin(), boid_vector.end(), makepos); */ 
    //eventualmente (SE VA!) mettere la lambda inline
    /*assert(std::all_of(boid_vector.begin(), boid_vector.end(), [&pos_vec] (boid& i_boid) { 
        int i = 0; 
        auto check = (i_boid.pos() == pos_vec[i]);
        return check;
        i++;})); */
        //questo assert fallisce, da capire se è lui ad essere sbagliato o il programma prima ON GOD E' LUI SBAGLIATO
         // i-esimo boid
    boid &i_boid = boid_vector[i];
    auto lambda = [&] (couple& rel, boid& j_boid) {
        if(BoidsAreNear(i_boid, j_boid, sep_dist, field_width, field_height)) {
            rel = rel + (j_boid.pos() - i_boid.pos());
        }
        return rel;
    };
    couple v_sep;
    v_sep = std::accumulate(boid_vector.begin(), boid_vector.end(), couple{0., 0.}, lambda); 
    /*couple v_sep;

    for (lu_int j = 0; j < boid_vector.size(); ++j)
    {
        boid j_boid = boid_vector[j];
        couple relative_pos = (boid_vector[j]).pos() - (boid_vector[i]).pos();
        if (j != i)
        {

            if (BoidsAreNear(i_boid, j_boid, sep_dist, field_width, field_height)) 
            {                                                                      
                v_sep = v_sep + relative_pos;
            }
        }
    } 
 */
    v_sep = (-1 * sep_fact) * v_sep;
    return v_sep;
} // regola di separazione
// ==> restituisce un array di coordinate di velocità

couple bds::v_alignment(lu_int i, double alig_fact, std::vector<boid> boid_vector)
{
    couple v_alig;
    assert(i < boid_vector.size());

    boid &i_boid = boid_vector[i];

    v_alig = std::accumulate(boid_vector.begin(), boid_vector.end(), couple {0., 0.}, [&] (couple& rvel, boid& j_boid) {
        if (i_boid != j_boid) { //capire se va bene come condizione di uguaglianza
            rvel = rvel + j_boid.vel();
        }
        return rvel;
    }); 

    /*for (lu_int j = 0; j < boid_vector.size(); ++j)
    { // nessun controllo vicini perchè la consegna non lo richiede
        if (j != i)
        {
            boid j_boid = boid_vector[j];
            v_alig = v_alig + j_boid.vel();
        }
    } */
    v_alig = (1. / static_cast<double>(boid_vector.size() - 1) * v_alig);
    v_alig = v_alig - i_boid.vel();

    v_alig = alig_fact * v_alig; 
    return v_alig;
} // regola di allineamento ==> restituisce un array di coordinate di velocità

couple bds::v_coesion(lu_int i, double dist_vic, double coes_fact, std::vector<boid> boid_vector, double field_width, double field_height)
{
    couple c_mass;
    assert(i < boid_vector.size());

    boid& i_boid = boid_vector[i];
    int near_boids{1};
    /*for (int k = 0; k < boid_vector.size(); ++k) {
        std::cout << "posizione boid " << k + 1 << ": " << boid_vector[k].pos()[0] << " " << boid_vector[k].pos()[1] << '\n';
    } */
    c_mass = std::accumulate(boid_vector.begin(), boid_vector.end(), couple{0., 0.}, [&] (couple& cm, boid& j_boid) {
        if (BoidsAreNear(i_boid, j_boid, dist_vic, field_width, field_height)) {
            cm = cm + j_boid.pos();
            ++near_boids;
        }
        return cm;
    }); 
    
    /*for (lu_int j = 0; j < boid_vector.size(); ++j)
    {
        boid j_boid = boid_vector[j]; 
        if (BoidsAreNear(i_boid, j_boid, dist_vic, field_width, field_height))
        {
            c_mass = c_mass + j_boid.pos();
            ++near_boids;
            std::cout << "vicino il boid n " << j << '\n';
            std::cout << "che ha posizione " << j_boid.pos()[0] << " " << j_boid.pos()[1] << '\n';
        }
    } */
    

    couple v_coes{0., 0.};
    if (near_boids == 1)
    {
        return v_coes;
    }
    else
    {
        c_mass = (1. / static_cast<double>(near_boids - 1)) * c_mass; // come prima static cast
        c_mass = c_mass - i_boid.pos();
        v_coes = coes_fact * c_mass;
        return v_coes;
    }
} // regola di coesione ==> restituisce un array di coordinate di velocità

couple bds::v_random()
{
    std::random_device rndm;
    std::default_random_engine eng(rndm());
    std::uniform_real_distribution<double> angle(0., 6.2830);
    std::uniform_real_distribution<double> vel(0., 1.);

    couple v_rndm{0., 0.};
    v_rndm[0] = 0.2 * cos(angle(eng));
    v_rndm[1] = 0.2 * sin(angle(eng)); // da capire se ha senso metterla in relazione alle altre

    return v_rndm;
}

void bds::v_mod(lu_int i, double sep_fact, double sep_dist, double alig_fact, double dist_vic, double coes_fact, std::vector<boid> &boid_vector, double field_width, double field_height)
{
    couple v_mod = v_separation(i, sep_dist, sep_fact, boid_vector, field_width, field_height) + v_alignment(i, alig_fact, boid_vector) + v_coesion(i, dist_vic, coes_fact, boid_vector,field_width, field_height) + v_random();
    boid_vector[i].vel_mod(v_mod);
    // la velocità del boid viene modificata dalla funzione vel_mod a cui viene dato in input l'array v_mod precedentemente "creato"
} // funzione che applica le modifiche di velocità legate alle regole al boid stesso, va iterata nel ciclo for per ogni boid

void bds::p_mod(lu_int i, std::vector<boid> &boid_vector, double deltat)
{
    boid_vector[i].pos_mod(deltat);
} // funzione che forse non è necessaria, può essere implementata direttamente nel ciclo for; modifica la posizione in base alle velocità

/*FUNZIONI CHE ESTRAGGONO VALORI STATISTICI*/

double bds::GetMeanDistance(std::vector<boid> boid_vector, double sep_dist, double field_width, double field_height)
{
    double dist_sum{0.};
    int n_events{0};

    dist_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0}, [&] (double& d_s, boid& i_boid) {
    d_s = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& ds, boid& j_boid) {
        if (BoidsAreNear(i_boid, j_boid, sep_dist, field_width, field_height)) {
            ds += sqrt(squaresum(i_boid.pos() - j_boid.pos()));
            ++n_events;
        }
        return ds;
    });
    return d_s;
    });

    dist_sum = dist_sum / 2; //doppio delle distanze sommate


    /*for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        for (lu_int j = 0; j < boid_vector.size(); ++j)
        {
            boid j_boid = boid_vector[j];
            if (BoidsAreNear(i_boid, j_boid, sep_dist, field_width, field_height))
            { // ho messo un controllo vicini qua perchè non credo abbia senso considerare nel calcolo della media la distanza tra i boid megalontani (ma forse lo ha...)
                dist_sum += sqrt(squaresum(i_boid.pos() - j_boid.pos()));
                ++n_events; // perchè non so a priori quante sono le coppie di boids che contribuiscono alla determinazione della distanza media
            }
        }
    } */
    double meandist = dist_sum / n_events;
    return meandist;
} // tira fuori la distanza media tra i boids di un dato vettore, da chiamare nel ciclo

double bds::GetMeanVelocity(std::vector<boid> boid_vector)
{
    double vel_sum{0.};

    vel_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& vs, boid& i_boid) {
        vs += sqrt(squaresum(i_boid.vel()));
        return vs;
    });

    /*for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        vel_sum += sqrt(squaresum(i_boid.vel()));
    } */
    double meanvel = vel_sum / boid_vector.size();
    return meanvel; 
} // tira fuori la velocità media dei boids, da chiamare nel ciclo

double bds::GetStdDevDistance(std::vector<boid> boid_vector, double sep_dist, double field_width, double field_height)
{
    double meandist = GetMeanDistance(boid_vector, sep_dist, field_width, field_height);
    double square_dist_sum{0.};
    int n_events{0};

    square_dist_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0}, [&] (double& d_s, boid& i_boid) {
    d_s = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& ds, boid& j_boid) {
        if (BoidsAreNear(i_boid, j_boid, sep_dist, field_width, field_height)) {
            ds += squaresum(i_boid.pos() - j_boid.pos());
            ++n_events;
        }
        return ds;
    });
    return d_s;
    });

    square_dist_sum = square_dist_sum / 2;

    /*for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        for (lu_int j = 0; j < boid_vector.size(); ++j)
        {
            boid j_boid = boid_vector[j];
            if (BoidsAreNear(i_boid, j_boid, sep_dist, field_width, field_height))
            {
                square_dist_sum += squaresum(i_boid.pos() - j_boid.pos());
                ++n_events;
            }
        }
    } */
    double stddev = (square_dist_sum / n_events) - (meandist * meandist);
    return stddev;
} // calcolo della deviazione standard come vuole la sara magica (quadrato della media - media dei quadrati)

double bds::GetStdDevVelocity(std::vector<boid> boid_vector)
{
    double square_vel_sum{0.};

    square_vel_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& svs, boid& i_boid) {
        svs += squaresum(i_boid.vel());
        return svs;
    });

    /*for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        square_vel_sum += squaresum(i_boid.vel()); 
    } */
    double meanvel = GetMeanVelocity(boid_vector);
    double stddev = (square_vel_sum / boid_vector.size()) - (meanvel * meanvel);
    return stddev;
} // come prima

void bds::Pacman(std::vector<bds::boid> &boid_vector, lu_int i, double field_width, double field_height)
{   

    periodize(boid_vector[i].pos()[0], field_width);
    periodize(boid_vector[i].pos()[1], field_height); //non va, perchè la funzione non agisce per reference
    // std::cout << "Before Pacman: Velocity = (" << boid_vector[i].vel()[0] << ", " << boid_vector[i].vel()[1] << ")\n";
   //  std::cout << "After Pacman: Velocity = (" << boid_vector[i].vel()[0] << ", " << boid_vector[i].vel()[1] << ")\n";
}

/*il programma compila senza warning ma ancora non so se funzioni
*/


